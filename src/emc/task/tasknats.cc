/********************************************************************
* Description: tasknats.cc
*   Task status -> NATS, Sparkplug B 3.0 (protobuf), next to the NML status
*   channel. See tasknats.hh for the contract.
*
*   The node itself - connection, NBIRTH/NDEATH, Rebirth, snapshot, devices hal
*   and scope of [NATS] PINS - is the halnats library (hal/utils/halnats), the
*   same code the halnats binary runs on VMs without LinuxCNC. The task is its
*   owner here and adds the status devices, one Sparkplug device per group of
*   Machinetalk's status service - task, motion, io, config, interp:
*     DBIRTH  spBv1//0.<group>.DBIRTH.<node>.<device>  full snapshot of the device
*     DDATA   spBv1//0.<group>.DDATA.<node>.<device>   changes only
*   Full snapshot on request: lcnc.<group>.<node>.snapshot (payload empty = all
*   devices, or a device name), metrics "<device>/<metric>".
*   Strings (task/file, task/command) go out as Sparkplug String.
*
*   Node = NODE_ID + "-task" while hal2spb publishes NODE_ID next to it (the
*   a3 comparison); INI [NATS] NODE overrides.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#include "config.h"
#include "tasknats.hh"

#include "libnml/rcs/rcs_print.hh"

#ifndef HAVE_LIBNATS

#include <inifile.hh>
#include <stdlib.h>

int taskNatsInit(const char *inifile, int, double, int)
{
    linuxcnc::IniFile ini(inifile);
    const char *env = getenv("SYNCTWIN_NATS_URL");
    if ((ini && ini.findString("URL", "NATS")) || (env && *env))
        rcs_print("task: a NATS URL is set but this milltask is built without libnats - "
                  "status goes to NML only\n");
    return 0;
}
void taskNatsCycle(const EMC_STAT *) {}
void taskNatsShutdown(void) {}

#else

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <stdint.h>
#include <string.h>

#include <inifile.hh>
#include "halnats.hh"
#include "nml_intf/emc.hh"
#include "nml_intf/emc_nml.hh"

using halnats::Val;
using halnats::Metric;
using halnats::Batch;
#define DT_(x) HN_DT(x)

namespace {

enum Dev { D_TASK, D_MOTION, D_IO, D_CONFIG, D_INTERP, D_N };
const std::vector<std::string> dev_names = {"task", "motion", "io", "config", "interp"};

struct Def {
    Dev dev;
    std::string name;
    uint32_t dt;                                 // DT_(Int32) | DT_(Boolean) | DT_(Double)
    std::function<Val(const EMC_STAT *)> get;
};

struct Change {                                  // one ring slot
    uint32_t idx;
    Val v;
    uint64_t ts_ms;
};

std::vector<Def> defs;

// task thread only: the last value pushed per metric
std::vector<Val> prev;
std::vector<char> prev_ok;

// SPSC ring: task thread writes head, publisher thread writes tail
std::vector<Change> ring;
size_t ring_mask = 0;
std::atomic<size_t> ring_head{0}, ring_tail{0};
std::atomic<uint64_t> dropped{0};
uint64_t dropped_said = 0;                       // publisher thread

// publisher side (under the node's lock)
std::vector<Val> mirror;
std::vector<uint64_t> mirror_ts;
std::vector<char> mirror_ok, dirty;
bool enabled = false;

// strings (task/file, task/command): the task thread hands a changed text over under smx,
// taken with try_lock only - busy = the change is retried next cycle, the task never waits
struct StrDef { Dev dev; std::string name; const char *(*get)(const EMC_STAT *); };
std::vector<StrDef> sdefs;
std::vector<std::string> sprev;                  // task thread
std::vector<char> sprev_ok;
std::mutex smx;
std::vector<std::string> sslot;                  // under smx
std::vector<uint64_t> sslot_ts;
std::vector<char> sslot_new;
std::vector<std::string> smirror;                // under the node's lock
std::vector<uint64_t> smirror_ts;
std::vector<char> smirror_ok, sdirty;
uint64_t str_alias0 = 0;                         // aliases: status 1..N, then strings

Val I(int64_t i) { Val v; v.i = i; return v; }
Val R(double d) { Val v; v.d = d; return v; }

void add(Dev d, std::string n, uint32_t dt, std::function<Val(const EMC_STAT *)> g)
{
    defs.push_back(Def{d, std::move(n), dt, std::move(g)});
}
#define AI(d, n, expr) add(d, n, DT_(Int32), [=](const EMC_STAT *s) { return I((int64_t)(expr)); })
#define AB(d, n, expr) add(d, n, DT_(Boolean), [=](const EMC_STAT *s) { return I((expr) ? 1 : 0); })
#define AR(d, n, expr) add(d, n, DT_(Double), [=](const EMC_STAT *s) { return R((double)(expr)); })

const char *const pose_name[9] = {"x", "y", "z", "a", "b", "c", "u", "v", "w"};
double pose_get(const EmcPose &p, int k)
{
    switch (k) {
    case 0: return p.tran.x; case 1: return p.tran.y; case 2: return p.tran.z;
    case 3: return p.a; case 4: return p.b; case 5: return p.c;
    case 6: return p.u; case 7: return p.v; default: return p.w;
    }
}
#define APOSE(d, base, member) \
    for (int k = 0; k < 9; k++) \
        AR(d, std::string(base) + "/" + pose_name[k], pose_get(s->member, k))

void build_defs(int joints, int spindles)
{
    // task
    AI(D_TASK, "mode", s->task.mode);
    AI(D_TASK, "state", s->task.state);
    AI(D_TASK, "execState", s->task.execState);
    AI(D_TASK, "interpState", s->task.interpState);
    AI(D_TASK, "status", s->task.status);
    AI(D_TASK, "echo_serial_number", s->echo_serial_number);
    AI(D_TASK, "callLevel", s->task.callLevel);
    AI(D_TASK, "motionLine", s->task.motionLine);
    AI(D_TASK, "currentLine", s->task.currentLine);
    AI(D_TASK, "readLine", s->task.readLine);
    AB(D_TASK, "optional_stop_state", s->task.optional_stop_state);
    AB(D_TASK, "block_delete_state", s->task.block_delete_state);
    AB(D_TASK, "input_timeout", s->task.input_timeout);
    AI(D_TASK, "task_paused", s->task.task_paused);
    AR(D_TASK, "delayLeft", s->task.delayLeft);
    AI(D_TASK, "queuedMDIcommands", s->task.queuedMDIcommands);
    sdefs.push_back(StrDef{D_TASK, "file", [](const EMC_STAT *s) { return (const char *)s->task.file; }});
    sdefs.push_back(StrDef{D_TASK, "command", [](const EMC_STAT *s) { return (const char *)s->task.command; }});

    // interp
    AI(D_INTERP, "interpreter_errcode", s->task.interpreter_errcode);
    AI(D_INTERP, "programUnits", s->task.programUnits);
    AI(D_INTERP, "g5x_index", s->task.g5x_index);
    APOSE(D_INTERP, "g5x_offset", task.g5x_offset);
    APOSE(D_INTERP, "g92_offset", task.g92_offset);
    APOSE(D_INTERP, "toolOffset", task.toolOffset);
    AR(D_INTERP, "rotation_xy", s->task.rotation_xy);
    for (int k = 0; k < ACTIVE_G_CODES; k++) AI(D_INTERP, "activeGCodes/" + std::to_string(k), s->task.activeGCodes[k]);
    for (int k = 0; k < ACTIVE_M_CODES; k++) AI(D_INTERP, "activeMCodes/" + std::to_string(k), s->task.activeMCodes[k]);
    for (int k = 0; k < ACTIVE_SETTINGS; k++) AR(D_INTERP, "activeSettings/" + std::to_string(k), s->task.activeSettings[k]);

    // motion
    AI(D_MOTION, "status", s->motion.status);
    AI(D_MOTION, "mode", s->motion.traj.mode);
    AB(D_MOTION, "enabled", s->motion.traj.enabled);
    AB(D_MOTION, "inpos", s->motion.traj.inpos);
    AI(D_MOTION, "queue", s->motion.traj.queue);
    AI(D_MOTION, "activeQueue", s->motion.traj.activeQueue);
    AB(D_MOTION, "queueFull", s->motion.traj.queueFull);
    AI(D_MOTION, "id", s->motion.traj.id);
    AB(D_MOTION, "paused", s->motion.traj.paused);
    AB(D_MOTION, "single_stepping", s->motion.traj.single_stepping);
    AR(D_MOTION, "scale", s->motion.traj.scale);
    AR(D_MOTION, "rapid_scale", s->motion.traj.rapid_scale);
    APOSE(D_MOTION, "position", motion.traj.position);
    APOSE(D_MOTION, "actualPosition", motion.traj.actualPosition);
    APOSE(D_MOTION, "probedPosition", motion.traj.probedPosition);
    APOSE(D_MOTION, "dtg", motion.traj.dtg);
    AB(D_MOTION, "probe_tripped", s->motion.traj.probe_tripped);
    AB(D_MOTION, "probing", s->motion.traj.probing);
    AI(D_MOTION, "probeval", s->motion.traj.probeval);
    AI(D_MOTION, "motion_type", s->motion.traj.motion_type);
    AR(D_MOTION, "distance_to_go", s->motion.traj.distance_to_go);
    AR(D_MOTION, "current_vel", s->motion.traj.current_vel);
    AB(D_MOTION, "feed_override_enabled", s->motion.traj.feed_override_enabled);
    AB(D_MOTION, "adaptive_feed_enabled", s->motion.traj.adaptive_feed_enabled);
    AB(D_MOTION, "feed_hold_enabled", s->motion.traj.feed_hold_enabled);
    AI(D_MOTION, "on_soft_limit", s->motion.on_soft_limit);
    AI(D_MOTION, "external_offsets_applied", s->motion.external_offsets_applied);
    for (int j = 0; j < joints; j++) {
        std::string b = "joint/" + std::to_string(j) + "/";
        AR(D_MOTION, b + "ferrorCurrent", s->motion.joint[j].ferrorCurrent);
        AR(D_MOTION, b + "ferrorHighMark", s->motion.joint[j].ferrorHighMark);
        AR(D_MOTION, b + "output", s->motion.joint[j].output);
        AR(D_MOTION, b + "input", s->motion.joint[j].input);
        AR(D_MOTION, b + "velocity", s->motion.joint[j].velocity);
        AB(D_MOTION, b + "inpos", s->motion.joint[j].inpos);
        AB(D_MOTION, b + "homing", s->motion.joint[j].homing);
        AB(D_MOTION, b + "homed", s->motion.joint[j].homed);
        AB(D_MOTION, b + "fault", s->motion.joint[j].fault);
        AB(D_MOTION, b + "enabled", s->motion.joint[j].enabled);
        AB(D_MOTION, b + "minSoftLimit", s->motion.joint[j].minSoftLimit);
        AB(D_MOTION, b + "maxSoftLimit", s->motion.joint[j].maxSoftLimit);
        AB(D_MOTION, b + "minHardLimit", s->motion.joint[j].minHardLimit);
        AB(D_MOTION, b + "maxHardLimit", s->motion.joint[j].maxHardLimit);
        AB(D_MOTION, b + "overrideLimits", s->motion.joint[j].overrideLimits);
    }
    for (int a = 0; a < EMCMOT_MAX_AXIS; a++)
        AR(D_MOTION, std::string("axis/") + pose_name[a] + "/velocity", s->motion.axis[a].velocity);
    for (int n = 0; n < spindles; n++) {
        std::string b = "spindle/" + std::to_string(n) + "/";
        AR(D_MOTION, b + "speed", s->motion.spindle[n].speed);
        AR(D_MOTION, b + "spindle_scale", s->motion.spindle[n].spindle_scale);
        AR(D_MOTION, b + "css_maximum", s->motion.spindle[n].css_maximum);
        AR(D_MOTION, b + "css_factor", s->motion.spindle[n].css_factor);
        AI(D_MOTION, b + "state", s->motion.spindle[n].state);
        AI(D_MOTION, b + "direction", s->motion.spindle[n].direction);
        AI(D_MOTION, b + "brake", s->motion.spindle[n].brake);
        AI(D_MOTION, b + "increasing", s->motion.spindle[n].increasing);
        AB(D_MOTION, b + "enabled", s->motion.spindle[n].enabled);
        AI(D_MOTION, b + "orient_state", s->motion.spindle[n].orient_state);
        AI(D_MOTION, b + "orient_fault", s->motion.spindle[n].orient_fault);
        AB(D_MOTION, b + "spindle_override_enabled", s->motion.spindle[n].spindle_override_enabled);
        AB(D_MOTION, b + "homed", s->motion.spindle[n].homed);
    }

    // io
    AI(D_IO, "status", s->io.status);
    AI(D_IO, "reason", s->io.reason);
    AI(D_IO, "fault", s->io.fault);
    AI(D_IO, "tool/pocketPrepped", s->io.tool.pocketPrepped);
    AI(D_IO, "tool/toolInSpindle", s->io.tool.toolInSpindle);
    AI(D_IO, "tool/toolFromPocket", s->io.tool.toolFromPocket);
    AB(D_IO, "coolant/mist", s->io.coolant.mist);
    AB(D_IO, "coolant/flood", s->io.coolant.flood);
    AB(D_IO, "aux/estop", s->io.aux.estop);

    // config
    AR(D_CONFIG, "linearUnits", s->motion.traj.linearUnits);
    AR(D_CONFIG, "angularUnits", s->motion.traj.angularUnits);
    AR(D_CONFIG, "cycleTime", s->motion.traj.cycleTime);
    AI(D_CONFIG, "joints", s->motion.traj.joints);
    AI(D_CONFIG, "spindles", s->motion.traj.spindles);
    AI(D_CONFIG, "axis_mask", s->motion.traj.axis_mask);
    AI(D_CONFIG, "kinematics_type", s->motion.traj.kinematics_type);
    AI(D_CONFIG, "switchkins_type", s->motion.traj.switchkins_type);
    AR(D_CONFIG, "velocity", s->motion.traj.velocity);
    AR(D_CONFIG, "acceleration", s->motion.traj.acceleration);
    AR(D_CONFIG, "maxVelocity", s->motion.traj.maxVelocity);
    AR(D_CONFIG, "maxAcceleration", s->motion.traj.maxAcceleration);
    for (int j = 0; j < joints; j++) {
        std::string b = "joint/" + std::to_string(j) + "/";
        AI(D_CONFIG, b + "jointType", s->motion.joint[j].jointType);
        AR(D_CONFIG, b + "units", s->motion.joint[j].units);
        AR(D_CONFIG, b + "backlash", s->motion.joint[j].backlash);
        AR(D_CONFIG, b + "minPositionLimit", s->motion.joint[j].minPositionLimit);
        AR(D_CONFIG, b + "maxPositionLimit", s->motion.joint[j].maxPositionLimit);
        AR(D_CONFIG, b + "maxFerror", s->motion.joint[j].maxFerror);
        AR(D_CONFIG, b + "minFerror", s->motion.joint[j].minFerror);
    }
    for (int a = 0; a < EMCMOT_MAX_AXIS; a++) {
        std::string b = std::string("axis/") + pose_name[a] + "/";
        AR(D_CONFIG, b + "minPositionLimit", s->motion.axis[a].minPositionLimit);
        AR(D_CONFIG, b + "maxPositionLimit", s->motion.axis[a].maxPositionLimit);
    }
}

// ---- the status devices as the node's owner (publisher thread, under the node's lock) ----

void status_metric(Batch &b, size_t i, bool named, bool prefixed)
{
    std::string n = named ? (prefixed ? dev_names[defs[i].dev] + "/" + defs[i].name : defs[i].name) : std::string();
    Metric &m = b.add(n, i + 1, named ? defs[i].dt : 0, mirror_ts[i]);
    halnats::set_value(&m, defs[i].dt, mirror[i]);
}
void str_metric(Batch &b, size_t k, bool named, bool prefixed)
{
    std::string n = named ? (prefixed ? dev_names[sdefs[k].dev] + "/" + sdefs[k].name : sdefs[k].name) : std::string();
    Metric &m = b.add(n, str_alias0 + k, named ? DT_(String) : 0, smirror_ts[k]);
    m.value_case = HN_V(STRING_VALUE); m.string_value = (char *)smirror[k].c_str();
}

struct TaskStatus : halnats::Owner {
    size_t aliases() const override { return defs.size() + sdefs.size(); }
    const std::vector<std::string> &devices() const override { return dev_names; }
    void pull() override
    {
        size_t t = ring_tail.load(std::memory_order_relaxed);
        size_t h = ring_head.load(std::memory_order_acquire);
        for (; t != h; t++) {
            const Change &c = ring[t & ring_mask];
            mirror[c.idx] = c.v; mirror_ts[c.idx] = c.ts_ms; mirror_ok[c.idx] = 1; dirty[c.idx] = 1;
        }
        ring_tail.store(t, std::memory_order_release);
        {
            std::lock_guard<std::mutex> sg(smx);     // the task only try_locks it
            for (size_t k = 0; k < sdefs.size(); k++)
                if (sslot_new[k]) {
                    smirror[k].swap(sslot[k]); smirror_ts[k] = sslot_ts[k];
                    smirror_ok[k] = 1; sdirty[k] = 1; sslot_new[k] = 0;
                }
        }
        uint64_t dr = dropped.load(std::memory_order_relaxed);
        if (dr != dropped_said) {
            rcs_print("task: NATS ring full - %llu status changes dropped so far (retried next cycle)\n",
                      (unsigned long long)dr);
            dropped_said = dr;
        }
    }
    bool ready() const override
    {
        for (char ok : mirror_ok) if (!ok) return false;
        for (char ok : smirror_ok) if (!ok) return false;
        return true;
    }
    void birth(size_t d, Batch &b) override
    {
        for (size_t i = 0; i < defs.size(); i++)
            if ((size_t)defs[i].dev == d && mirror_ok[i]) { status_metric(b, i, true, false); dirty[i] = 0; }
        for (size_t k = 0; k < sdefs.size(); k++)
            if ((size_t)sdefs[k].dev == d && smirror_ok[k]) { str_metric(b, k, true, false); sdirty[k] = 0; }
    }
    void data(size_t d, Batch &b) override
    {
        for (size_t i = 0; i < defs.size(); i++)
            if ((size_t)defs[i].dev == d && dirty[i]) { status_metric(b, i, false, false); dirty[i] = 0; }
        for (size_t k = 0; k < sdefs.size(); k++)
            if ((size_t)sdefs[k].dev == d && sdirty[k]) { str_metric(b, k, false, false); sdirty[k] = 0; }
    }
    void snapshot(const std::string &want, Batch &b) override
    {
        for (size_t i = 0; i < defs.size(); i++)
            if (mirror_ok[i] && (want.empty() || want == dev_names[defs[i].dev])) status_metric(b, i, true, true);
        for (size_t k = 0; k < sdefs.size(); k++)
            if (smirror_ok[k] && (want.empty() || want == dev_names[sdefs[k].dev])) str_metric(b, k, true, true);
    }
};
TaskStatus owner;

void log_line(bool error, const char *ln)
{
    // rcs_print_error is two statements (rcs_print.hh): braces, not a bare if/else
    if (error) { rcs_print_error("%s", ln); }
    else { rcs_print("%s", ln); }
}

} // namespace

int taskNatsInit(const char *inifile, int joints, double task_cycle_time, int hal_comp_id)
{
    halnats::Config c;
    c.tag = "task";
    c.log = log_line;
    // the publisher's period: [NATS] PUBLISH_PERIOD, else the task cycle (one publish per task cycle at most)
    int r = halnats::configure(c, inifile, "-task", task_cycle_time);
    if (r) return r > 0 ? 0 : -1;

    // [TRAJ] SPINDLES: LinuxCNC's own default is 1 (motion's num_spindles)
    linuxcnc::IniFile ini(inifile);
    int spindles = ini ? ini.findSIntV("SPINDLES", "TRAJ", 1) : 1;
    if (joints < 0 || joints > EMCMOT_MAX_JOINTS) joints = EMCMOT_MAX_JOINTS;
    if (spindles < 0 || spindles > EMCMOT_MAX_SPINDLES) spindles = EMCMOT_MAX_SPINDLES;

    build_defs(joints, spindles);
    size_t n = defs.size();
    size_t ns = sdefs.size();
    sprev.assign(ns, std::string()); sprev_ok.assign(ns, 0);
    sslot.assign(ns, std::string()); sslot_ts.assign(ns, 0); sslot_new.assign(ns, 0);
    smirror.assign(ns, std::string()); smirror_ts.assign(ns, 0); smirror_ok.assign(ns, 0); sdirty.assign(ns, 0);
    str_alias0 = n + 1;
    prev.assign(n, Val{}); prev_ok.assign(n, 0);
    mirror.assign(n, Val{}); mirror_ts.assign(n, 0); mirror_ok.assign(n, 0); dirty.assign(n, 0);
    // ring capacity: four full snapshots, rounded up to a power of two
    // (ПРЕДПОЛОЖЕНИЕ, не замер: 4 snapshots cover a publisher lagging 4 periods)
    size_t cap = 1;
    while (cap < 4 * n) cap <<= 1;
    ring.assign(cap, Change{}); ring_mask = cap - 1;

    if (halnats::start(c, hal_comp_id, &owner)) return -1;
    enabled = true;
    return 0;
}

// Task main loop, right after the NML status write. Never blocks.
void taskNatsCycle(const EMC_STAT *stat)
{
    if (!enabled) return;
    uint64_t ts = halnats::now_ms();
    size_t h = ring_head.load(std::memory_order_relaxed);
    size_t t = ring_tail.load(std::memory_order_acquire);
    for (size_t i = 0; i < defs.size(); i++) {
        Val v = defs[i].get(stat);
        if (prev_ok[i] && memcmp(&v, &prev[i], sizeof v) == 0) continue;
        if (h - t > ring_mask) {                      // full: count, keep prev so it is retried next cycle
            dropped.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        ring[h & ring_mask] = Change{(uint32_t)i, v, ts};
        h++;
        prev[i] = v; prev_ok[i] = 1;
    }
    ring_head.store(h, std::memory_order_release);
    // strings: compared here, handed over only if the publisher is not holding the slots right now
    for (size_t k = 0; k < sdefs.size(); k++) {
        const char *v = sdefs[k].get(stat);
        if (sprev_ok[k] && sprev[k] == v) continue;
        std::unique_lock<std::mutex> g(smx, std::try_to_lock);
        if (!g.owns_lock()) break;                    // busy: retried next cycle
        sslot[k] = v; sslot_ts[k] = ts; sslot_new[k] = 1;
        sprev[k] = v; sprev_ok[k] = 1;
    }
}

void taskNatsShutdown(void)
{
    if (!enabled) return;
    enabled = false;
    halnats::stop();
}

#endif
