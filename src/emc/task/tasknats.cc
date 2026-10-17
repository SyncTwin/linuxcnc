/********************************************************************
* Description: tasknats.cc
*   Task status -> NATS, Sparkplug B 3.0 (protobuf), next to the NML status
*   channel. See tasknats.hh for the contract.
*
*   Topics follow Machinetalk's status service: one Sparkplug device per
*   group - task, motion, io, config, interp - of the edge node
*   [NATS] GROUP / NODE:
*     NBIRTH/NDEATH  spBv1//0.<group>.NBIRTH.<node>          (MQTT: spBv1.0/<group>/NBIRTH/<node>)
*     DBIRTH         spBv1//0.<group>.DBIRTH.<node>.<device>  full snapshot of the device
*     DDATA          spBv1//0.<group>.DDATA.<node>.<device>   changes only
*   Full snapshot on request (NATS request-reply): lcnc.<group>.<node>.snapshot
*   (payload empty = all devices, or a device name) -> one payload, metrics named
*   "<device>/<metric>" with datatype.
*   Subjects are the NATS spelling of the Sparkplug MQTT topics (the nats-server
*   MQTT gateway maps '/' <-> '.' and '.' <-> '//'), so MQTT Sparkplug hosts
*   see them unchanged.
*
*   Strings (task.file, task.command) are not published: the Sparkplug schema
*   shared with hal2spb declares no string_value yet.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#include "config.h"
#include "tasknats.hh"

#include <inifile.hh>
#include "libnml/rcs/rcs_print.hh"

#ifndef HAVE_LIBNATS

int taskNatsInit(const char *inifile, int, double)
{
    IniFile ini(inifile);
    if (ini && ini.findString("URL", "NATS"))
        rcs_print("task: [NATS] URL is set but this milltask is built without libnats - "
                  "status goes to NML only\n");
    return 0;
}
void taskNatsCycle(const EMC_STAT *) {}
void taskNatsShutdown(void) {}

#else

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <nats/nats.h>
#include "nml_intf/emc.hh"
#include "nml_intf/emc_nml.hh"
#include "sparkplug_b_payload.pb-c.h"

typedef SparkplugB__Payload Payload;
typedef SparkplugB__Payload__Metric Metric;
#define V_(x) SPARKPLUG_B__PAYLOAD__METRIC__VALUE_##x
#define DT_(x) SPARKPLUG_B__DATA_TYPE__##x   /* Sparkplug B 3.0, section 6.4.16 */

namespace {

enum Dev { D_TASK, D_MOTION, D_IO, D_CONFIG, D_INTERP, D_N };
const char *const dev_name[D_N] = {"task", "motion", "io", "config", "interp"};

union Val { int64_t i; double d; };

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

// publisher side (publisher thread + nats callback thread under mx)
std::mutex mx;
std::vector<Val> mirror;
std::vector<uint64_t> mirror_ts;
std::vector<char> mirror_ok, dirty;
natsConnection *nc = nullptr;
natsSubscription *snap_sub = nullptr;
std::thread pub_thread;
std::atomic<bool> running{false}, need_birth{false};
bool enabled = false;
int64_t bdseq = 0;
uint64_t seq = 0;
double period_s = 0;
std::string s_nbirth, s_ndeath, s_dbirth[D_N], s_ddata[D_N], s_snapshot;

uint64_t now_ms()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return (uint64_t)t.tv_sec * 1000u + (uint64_t)(t.tv_nsec / 1000000);
}

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

// ---- Sparkplug encoding (publisher side, under mx) ----

void m_value(Metric *m, uint32_t dt, Val v)
{
    if (dt == DT_(Double)) { m->value_case = V_(DOUBLE_VALUE); m->double_value = v.d; }
    else if (dt == DT_(Boolean)) { m->value_case = V_(BOOLEAN_VALUE); m->boolean_value = v.i != 0; }
    else { m->value_case = V_(INT_VALUE); m->int_value = (uint32_t)(int32_t)v.i; }   // Int32 two's complement
}

void send(const std::string &subj, const std::string *reply, std::vector<Metric> &ms, bool with_seq)
{
    std::vector<Metric *> mp(ms.size());
    for (size_t k = 0; k < ms.size(); k++) mp[k] = &ms[k];
    Payload pl = SPARKPLUG_B__PAYLOAD__INIT;
    pl.has_timestamp = 1; pl.timestamp = now_ms();
    pl.n_metrics = mp.size(); pl.metrics = mp.data();
    if (with_seq) { pl.has_seq = 1; pl.seq = seq; seq = (seq + 1) & 0xff; }
    size_t n = sparkplug_b__payload__get_packed_size(&pl);
    std::vector<uint8_t> buf(n);
    sparkplug_b__payload__pack(&pl, buf.data());
    natsConnection_Publish(nc, reply ? reply->c_str() : subj.c_str(), buf.data(), (int)n);
}

// full metric (name, alias, datatype, value); prefix "<device>/" for the snapshot reply
Metric full(size_t i, std::vector<std::string> &names, bool prefixed)
{
    Metric m = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
    names.push_back(prefixed ? std::string(dev_name[defs[i].dev]) + "/" + defs[i].name : defs[i].name);
    m.name = (char *)names.back().c_str();
    m.has_alias = 1; m.alias = i + 1;
    m.has_datatype = 1; m.datatype = defs[i].dt;
    m.has_timestamp = 1; m.timestamp = mirror_ts[i];
    m_value(&m, defs[i].dt, mirror[i]);
    return m;
}

void births()
{
    seq = 0;
    const Metric m0 = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
    std::vector<Metric> ms(2, m0);
    ms[0].name = (char *)"bdSeq"; ms[0].has_datatype = 1; ms[0].datatype = DT_(Int64);
    ms[0].value_case = V_(LONG_VALUE); ms[0].long_value = bdseq;
    ms[1].name = (char *)"Node Control/Rebirth"; ms[1].has_datatype = 1; ms[1].datatype = DT_(Boolean);
    ms[1].value_case = V_(BOOLEAN_VALUE); ms[1].boolean_value = 0;
    send(s_nbirth, nullptr, ms, true);
    for (int d = 0; d < D_N; d++) {
        std::vector<Metric> dm; std::vector<std::string> names;
        names.reserve(defs.size());
        for (size_t i = 0; i < defs.size(); i++)
            if (defs[i].dev == d && mirror_ok[i]) { dm.push_back(full(i, names, false)); dirty[i] = 0; }
        send(s_dbirth[d], nullptr, dm, true);
    }
}

void datas()
{
    for (int d = 0; d < D_N; d++) {
        std::vector<Metric> dm;
        for (size_t i = 0; i < defs.size(); i++) {
            if (defs[i].dev != d || !dirty[i]) continue;
            Metric m = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
            m.has_alias = 1; m.alias = i + 1;            // name and datatype were told in DBIRTH
            m.has_timestamp = 1; m.timestamp = mirror_ts[i];
            m_value(&m, defs[i].dt, mirror[i]);
            dm.push_back(m); dirty[i] = 0;
        }
        if (!dm.empty()) send(s_ddata[d], nullptr, dm, true);
    }
}

void on_snapshot(natsConnection *, natsSubscription *, natsMsg *msg, void *)
{
    const char *reply = natsMsg_GetReply(msg);
    if (reply && *reply) {
        std::string want(natsMsg_GetData(msg) ? natsMsg_GetData(msg) : "", natsMsg_GetDataLength(msg));
        std::string r(reply);
        std::lock_guard<std::mutex> g(mx);
        std::vector<Metric> ms; std::vector<std::string> names;
        names.reserve(defs.size());
        for (size_t i = 0; i < defs.size(); i++)
            if (mirror_ok[i] && (want.empty() || want == dev_name[defs[i].dev]))
                ms.push_back(full(i, names, true));
        send(r, &r, ms, false);                      // a reply, not part of the seq stream
    }
    natsMsg_Destroy(msg);
}

void on_connected(natsConnection *, void *)
{
    rcs_print("task: NATS connected - Sparkplug births follow\n");
    need_birth = true;
}
void on_disconnected(natsConnection *, void *) { rcs_print("task: NATS disconnected\n"); }

void publisher()
{
    uint64_t dropped_said = 0;
    while (running) {
        {
            std::lock_guard<std::mutex> g(mx);
            size_t t = ring_tail.load(std::memory_order_relaxed);
            size_t h = ring_head.load(std::memory_order_acquire);
            for (; t != h; t++) {
                const Change &c = ring[t & ring_mask];
                mirror[c.idx] = c.v; mirror_ts[c.idx] = c.ts_ms; mirror_ok[c.idx] = 1; dirty[c.idx] = 1;
            }
            ring_tail.store(t, std::memory_order_release);
            if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) {
                bool all = true;
                for (char ok : mirror_ok) all = all && ok;
                if (need_birth && all) { need_birth = false; births(); }
                else if (!need_birth) datas();
            }
        }
        uint64_t dr = dropped.load(std::memory_order_relaxed);
        if (dr != dropped_said) {
            rcs_print("task: NATS ring full - %llu status changes dropped so far (retried next cycle)\n",
                      (unsigned long long)dr);
            dropped_said = dr;
        }
        struct timespec p = {(time_t)period_s, (long)((period_s - (time_t)period_s) * 1e9)};
        nanosleep(&p, nullptr);
    }
}

bool name_ok(const std::string &s)
{
    if (s.empty()) return false;
    for (char c : s)
        if (c == '.' || c == '/' || c == '+' || c == '#' || c == '*' || c == '>' || c <= ' ') return false;
    return true;
}

} // namespace

int taskNatsInit(const char *inifile, int joints, double task_cycle_time)
{
    IniFile ini(inifile);
    std::optional<std::string> url;
    if (ini) url = ini.findString("URL", "NATS");
    if (!url) {
        rcs_print("task: no [NATS] URL - status publishing to NATS is off, NML as before\n");
        return 0;
    }
    auto group = ini.findString("GROUP", "NATS");
    auto node = ini.findString("NODE", "NATS");
    if (!group || !node || !name_ok(*group) || !name_ok(*node)) {
        rcs_print_error("task: [NATS] URL is set but GROUP/NODE is missing or not a Sparkplug id "
                        "(no . / + # * > or spaces) - NATS publishing refused, NML as before\n");
        return -1;
    }
    // the publisher's period: [NATS] PUBLISH_PERIOD, else the task cycle (one publish per task cycle at most)
    period_s = ini.findRealV("PUBLISH_PERIOD", "NATS", task_cycle_time);
    if (!(period_s > 0)) {
        rcs_print_error("task: [NATS] PUBLISH_PERIOD %f is not > 0 - NATS publishing refused\n", period_s);
        return -1;
    }
    // [TRAJ] SPINDLES: LinuxCNC's own default is 1 (motion's num_spindles)
    int spindles = ini.findSIntV("SPINDLES", "TRAJ", 1);
    if (joints < 0 || joints > EMCMOT_MAX_JOINTS) joints = EMCMOT_MAX_JOINTS;
    if (spindles < 0 || spindles > EMCMOT_MAX_SPINDLES) spindles = EMCMOT_MAX_SPINDLES;

    build_defs(joints, spindles);
    size_t n = defs.size();
    prev.assign(n, Val{}); prev_ok.assign(n, 0);
    mirror.assign(n, Val{}); mirror_ts.assign(n, 0); mirror_ok.assign(n, 0); dirty.assign(n, 0);
    // ring capacity: four full snapshots, rounded up to a power of two
    // (ПРЕДПОЛОЖЕНИЕ, не замер: 4 snapshots cover a publisher lagging 4 periods)
    size_t cap = 1;
    while (cap < 4 * n) cap <<= 1;
    ring.assign(cap, Change{}); ring_mask = cap - 1;

    const std::string base = "spBv1//0." + *group + ".";
    s_nbirth = base + "NBIRTH." + *node;
    s_ndeath = base + "NDEATH." + *node;
    for (int d = 0; d < D_N; d++) {
        s_dbirth[d] = base + "DBIRTH." + *node + "." + dev_name[d];
        s_ddata[d] = base + "DDATA." + *node + "." + dev_name[d];
    }
    s_snapshot = "lcnc." + *group + "." + *node + ".snapshot";
    bdseq = (int64_t)(now_ms() & 0xff);  // a fresh bdSeq per run (no persisted counter yet)

    natsOptions *opts = nullptr;
    natsStatus st = natsOptions_Create(&opts);
    if (st == NATS_OK) st = natsOptions_SetURL(opts, url->c_str());
    if (st == NATS_OK) st = natsOptions_SetMaxReconnect(opts, -1);
    // connect in the background: the task never waits on the network, not even at startup
    if (st == NATS_OK) st = natsOptions_SetRetryOnFailedConnect(opts, true, on_connected, nullptr);
    if (st == NATS_OK) st = natsOptions_SetReconnectedCB(opts, on_connected, nullptr);
    if (st == NATS_OK) st = natsOptions_SetDisconnectedCB(opts, on_disconnected, nullptr);
    if (st == NATS_OK) {
        st = natsConnection_Connect(&nc, opts);
        if (st == NATS_NOT_YET_CONNECTED) st = NATS_OK;
    }
    if (st == NATS_OK) st = natsConnection_Subscribe(&snap_sub, nc, s_snapshot.c_str(), on_snapshot, nullptr);
    natsOptions_Destroy(opts);
    if (st != NATS_OK) {
        rcs_print_error("task: NATS %s: %s - NATS publishing refused, NML as before\n",
                        url->c_str(), natsStatus_GetText(st));
        natsSubscription_Destroy(snap_sub); snap_sub = nullptr;
        natsConnection_Destroy(nc); nc = nullptr;
        return -1;
    }
    if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) need_birth = true;
    enabled = true;
    running = true;
    pub_thread = std::thread(publisher);
    rcs_print("task: status -> NATS %s as Sparkplug node %s/%s, %zu metrics, snapshot on %s\n",
              url->c_str(), group->c_str(), node->c_str(), n, s_snapshot.c_str());
    return 0;
}

// Task main loop, right after the NML status write. Never blocks.
void taskNatsCycle(const EMC_STAT *stat)
{
    if (!enabled) return;
    uint64_t ts = now_ms();
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
}

void taskNatsShutdown(void)
{
    if (!enabled) return;
    enabled = false;
    running = false;
    if (pub_thread.joinable()) pub_thread.join();
    {
        std::lock_guard<std::mutex> g(mx);
        const Metric m0 = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
        std::vector<Metric> ms(1, m0);
        ms[0].name = (char *)"bdSeq"; ms[0].has_datatype = 1; ms[0].datatype = DT_(Int64);
        ms[0].value_case = V_(LONG_VALUE); ms[0].long_value = bdseq;
        if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) {
            send(s_ndeath, nullptr, ms, false);       // NDEATH carries no seq (6.4.5)
            // ПРЕДПОЛОЖЕНИЕ, не замер: 1 s for NDEATH to leave at shutdown
            natsConnection_FlushTimeout(nc, 1000);
        }
    }
    natsSubscription_Destroy(snap_sub); snap_sub = nullptr;
    natsConnection_Destroy(nc); nc = nullptr;
}

#endif
