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
*   Strings (task/file, task/command) go out as Sparkplug String (string_value).
*
*   Two more devices of the same node, both read in the publisher's threads
*   (never in the task loop), from the file [NATS] PINS - the hal2spb.conf
*   line format (appliance/hal2spb), which this replaces on the VM:
*     hal    pin <key> bool|sint|uint|real [dir=in|out] src=<pin> | sig=<signal>
*                [q=<quantum>] [db=<deadband>]
*            Read by name through the HAL query API (hal_get_p / hal_get_s, the
*            link is resolved at every read, so a `net` after the task started
*            is followed); task makes no pins of its own - HALFILEs run after
*            the task, so a source that is not there yet is waited for and the
*            DBIRTH of `hal` comes when every source reads. Metric = key with
*            '.' -> '/'. Real: published round(v/q)*q when |v - last_sent| >= db
*            (db defaults to q); bits/ints: any change (or > db). dir=out: DCMD
*            spBv1//0.<group>.DCMD.<node>.hal writes it by name or alias, with
*            halcmd's rules (sig=: a signal without a writer, `sets`; src=: an
*            unlinked IN pin, `setp`); a message is checked whole before any
*            write; the value goes out read back by the next DDATA.
*            hz <f>: read rate of these sources (else every publisher period);
*            spb_keepalive_s <s>: an empty DDATA of `hal` after s of silence.
*     scope  scope_channel <n> | scope_batch <n> | scope_dt_ns <n> |
*            scope_type double|float | scope_col <Key.chan> [scale]
*            The FIFO of `sampler` channel n (hal_stream, as halsampler):
*            attached once the sampler component exists, the tail queued at
*            attach is dropped, then batches of scope_batch rows go out as one
*            DDATA of DoubleArray/FloatArray columns (packed LE), stamped with
*            the first row's time; a column constant at its last sent value is
*            left out (report by exception).
*   NCMD "Node Control/Rebirth" = true -> all births again.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#include "config.h"
#include "tasknats.hh"

#include <inifile.hh>
#include "libnml/rcs/rcs_print.hh"

using linuxcnc::IniFile;

// rcs_print_error is two statements (rcs_print.hh): one statement here, safe under a bare if/else
#define TN_ERR(...) do { rcs_print_error(__VA_ARGS__); } while (0)

#ifndef HAVE_LIBNATS

int taskNatsInit(const char *inifile, int, double, int)
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
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <nats/nats.h>
#include "hal.h"
#include "../../hal/components/streamer.h"   // SAMPLER_SHMEM_KEY - sampler.c's FIFO key
#include "nml_intf/emc.hh"
#include "nml_intf/emc_nml.hh"
#include "sparkplug_b_payload.pb-c.h"

typedef SparkplugB__Payload Payload;
typedef SparkplugB__Payload__Metric Metric;
#define V_(x) SPARKPLUG_B__PAYLOAD__METRIC__VALUE_##x
#define DT_(x) SPARKPLUG_B__DATA_TYPE__##x   /* Sparkplug B 3.0, section 6.4.16 */

namespace {

// D_TASK..D_INTERP: the status devices (EMC_STAT); D_HAL, D_SCOPE: the [NATS] PINS file
enum Dev { D_TASK, D_MOTION, D_IO, D_CONFIG, D_INTERP, D_STATUS_N, D_HAL = D_STATUS_N, D_SCOPE, D_N };
const char *const dev_name[D_N] = {"task", "motion", "io", "config", "interp", "hal", "scope"};

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
natsSubscription *snap_sub = nullptr, *ncmd_sub = nullptr, *dcmd_sub = nullptr;
std::thread pub_thread, scope_thread;
std::atomic<bool> running{false}, need_birth{false};
bool enabled = false;
bool born = false;                               // under mx: births of this connection sent
int64_t bdseq = 0;
uint64_t seq = 0;
double period_s = 0;
double last_pub = 0;                             // under mx: CLOCK_MONOTONIC of the last seq message
std::string s_nbirth, s_ndeath, s_ncmd, s_dcmd_hal, s_dbirth[D_N], s_ddata[D_N], s_snapshot;

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
std::vector<std::string> smirror;                // under mx
std::vector<uint64_t> smirror_ts;
std::vector<char> smirror_ok, sdirty;

// device hal: pins / signals by name (publisher thread under mx; DCMD on the nats thread)
struct HPin {
    std::string key, metric;                     // conf key; Sparkplug name ('.' -> '/')
    hal_type_t type;
    bool out;                                    // dir=out: DCMD writes it
    bool sig;                                    // sig=<signal>, else src=<pin>
    std::string src;
    double q, db;
    Val v, last;                                 // last read / last sent (quantized)
    uint64_t ts;
    bool ok, sent, said;
};
std::vector<HPin> hpins;
bool hal_born = false;                           // under mx
double keepalive_s = 0;                          // spb_keepalive_s of the file; 0 = none
double hal_period_s = 0;                         // 1 / hz of the file; 0 = every publisher period
double hal_last_read = 0;

// device scope: the sampler FIFO
int s_chan = -1, s_batch = 0;
long s_dt_ns = 0;
bool s_float = false;
struct SCol { std::string key, ch; double scale; };
std::vector<SCol> scols;
int hal_comp = -1;

// aliases, unique over the node: status 1..N, strings, hal, scope
uint64_t str_alias0 = 0, hal_alias0 = 0, scope_alias0 = 0;

uint64_t now_ms()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return (uint64_t)t.tv_sec * 1000u + (uint64_t)(t.tv_nsec / 1000000);
}
double now_mono()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
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

// ---- Sparkplug encoding (publisher side, under mx) ----

void m_value(Metric *m, uint32_t dt, Val v)
{
    if (dt == DT_(Double)) { m->value_case = V_(DOUBLE_VALUE); m->double_value = v.d; }
    else if (dt == DT_(Boolean)) { m->value_case = V_(BOOLEAN_VALUE); m->boolean_value = v.i != 0; }
    else if (dt == DT_(Int64) || dt == DT_(UInt64)) { m->value_case = V_(LONG_VALUE); m->long_value = (uint64_t)v.i; }
    else { m->value_case = V_(INT_VALUE); m->int_value = (uint32_t)(int32_t)v.i; }   // Int32 two's complement
}

void send(const std::string &subj, const std::string *reply, std::vector<Metric> &ms, bool with_seq)
{
    std::vector<Metric *> mp(ms.size());
    for (size_t k = 0; k < ms.size(); k++) mp[k] = &ms[k];
    Payload pl = SPARKPLUG_B__PAYLOAD__INIT;
    pl.has_timestamp = 1; pl.timestamp = now_ms();
    pl.n_metrics = mp.size(); pl.metrics = mp.data();
    if (with_seq) { pl.has_seq = 1; pl.seq = seq; seq = (seq + 1) & 0xff; last_pub = now_mono(); }
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
Metric str_metric(size_t k, std::vector<std::string> *names, bool prefixed)
{
    Metric m = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
    if (names) {
        names->push_back(prefixed ? std::string(dev_name[sdefs[k].dev]) + "/" + sdefs[k].name : sdefs[k].name);
        m.name = (char *)names->back().c_str();
        m.has_datatype = 1; m.datatype = DT_(String);
    }
    m.has_alias = 1; m.alias = str_alias0 + k;
    m.has_timestamp = 1; m.timestamp = smirror_ts[k];
    m.value_case = V_(STRING_VALUE); m.string_value = (char *)smirror[k].c_str();
    return m;
}

// ---- device hal ----

uint32_t hal_dt(hal_type_t t)
{
    return t == HAL_BOOL ? DT_(Boolean) : t == HAL_REAL ? DT_(Double) : t == HAL_UINT ? DT_(UInt64) : DT_(Int64);
}
Val quant(const HPin &p, Val v)
{
    if (p.type == HAL_REAL && p.q > 0) v.d = std::round(v.d / p.q) * p.q;
    return v;
}
// Deadband with hysteresis (hal2spb): the RAW value against the last SENT (quantized) one, so noise
// straddling a rounding boundary does not flip the published value; a change of exactly db goes out.
bool moved(const HPin &p, Val v)
{
    if (!p.sent) return true;
    if (p.type != HAL_REAL) {                    // bits/ints: exact distance in integers
        uint64_t a = (uint64_t)v.i, b = (uint64_t)p.last.i;
        bool up = p.type == HAL_UINT ? a > b : v.i > p.last.i;
        return (double)(up ? a - b : b - a) > p.db;
    }
    double d = std::fabs(v.d - p.last.d);         // a NaN read counts as a change
    if (std::isnan(d)) return true;
    return p.q > 0 ? d + p.q * 1e-6 >= p.db : d > p.db;
}
// one read through the HAL query API: under the HAL mutex, the link resolved at this call
int hal_read(const HPin &p, Val &v)
{
    hal_query_t q;
    memset(&q, 0, sizeof q);
    q.name = p.src.c_str();
    hal_query_value_u u;
    int rc;
    if (p.sig) { q.qtype = HAL_QTYPE_SIGNAL; q.sig.type = p.type; rc = hal_get_s(&q, nullptr, nullptr); u = q.sig.value; }
    else { q.qtype = HAL_QTYPE_PIN; q.pp.type = p.type; rc = hal_get_p(&q, nullptr, nullptr); u = q.pp.value; }
    if (rc) return rc;
    switch (p.type) {
    case HAL_BOOL: v.i = u.b ? 1 : 0; break;
    case HAL_SINT: v.i = u.s; break;
    case HAL_UINT: v.i = (int64_t)u.u; break;
    default: v.d = u.r; break;
    }
    return 0;
}
const char *hal_err(int rc)
{
    return rc == -ENOENT ? "not in HAL (yet)" : rc == -EEXIST ? "HAL type differs from the conf"
         : rc == -EACCES ? "HAL refused the write (signal has a writer / pin is OUT or linked)" : strerror(-rc);
}
void hal_poll()
{
    uint64_t ts = now_ms();
    for (HPin &p : hpins) {
        Val v{};
        int rc = hal_read(p, v);
        if (rc) {
            if (!p.said) rcs_print("task: NATS hal %s %s: %s - DBIRTH of hal waits\n",
                                   p.sig ? "signal" : "pin", p.src.c_str(), hal_err(rc));
            p.said = true;
            continue;
        }
        if (p.said && !p.ok) rcs_print("task: NATS hal %s now reads\n", p.src.c_str());
        p.v = v; p.ts = ts; p.ok = true;
    }
}
bool hal_all_ok()
{
    for (const HPin &p : hpins) if (!p.ok) return false;
    return true;
}
Metric hal_metric(size_t k, std::vector<std::string> *names, bool prefixed)
{
    HPin &p = hpins[k];
    Metric m = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
    if (names) {
        names->push_back(prefixed ? "hal/" + p.metric : p.metric);
        m.name = (char *)names->back().c_str();
        m.has_datatype = 1; m.datatype = hal_dt(p.type);
    }
    m.has_alias = 1; m.alias = hal_alias0 + k;
    m.has_timestamp = 1; m.timestamp = p.ts;
    m_value(&m, hal_dt(p.type), quant(p, p.v));
    return m;
}
void hal_birth()
{
    std::vector<Metric> ms; std::vector<std::string> names;
    names.reserve(hpins.size());
    for (size_t k = 0; k < hpins.size(); k++) {
        ms.push_back(hal_metric(k, &names, false));
        hpins[k].last = quant(hpins[k], hpins[k].v); hpins[k].sent = true;
    }
    send(s_dbirth[D_HAL], nullptr, ms, true);
    hal_born = true;
}
void hal_datas()
{
    std::vector<Metric> ms;
    for (size_t k = 0; k < hpins.size(); k++) {
        HPin &p = hpins[k];
        if (!p.ok || !moved(p, p.v)) continue;
        ms.push_back(hal_metric(k, nullptr, false));
        p.last = quant(p, p.v); p.sent = true;
    }
    // empty DDATA = keepalive: a node standing still is not silent past spb_keepalive_s
    if (!ms.empty() || (keepalive_s > 0 && now_mono() - last_pub >= keepalive_s))
        send(s_ddata[D_HAL], nullptr, ms, true);
}

// ---- device scope (declared in births, data from scope_main) ----

void scope_birth()
{
    const Metric m0 = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
    std::vector<Metric> ms(1 + scols.size(), m0);
    std::vector<std::string> names; names.reserve(scols.size());
    ms[0].name = (char *)"dt_ns"; ms[0].has_alias = 1; ms[0].alias = scope_alias0;
    ms[0].has_datatype = 1; ms[0].datatype = DT_(Int64);
    ms[0].value_case = V_(LONG_VALUE); ms[0].long_value = (uint64_t)s_dt_ns;
    for (size_t c = 0; c < scols.size(); c++) {  // declared, empty array: values come in DDATA
        names.push_back(scols[c].key + "/" + scols[c].ch);
        Metric &m = ms[1 + c];
        m.name = (char *)names.back().c_str(); m.has_alias = 1; m.alias = scope_alias0 + 1 + c;
        m.has_datatype = 1; m.datatype = s_float ? DT_(FloatArray) : DT_(DoubleArray);
        m.value_case = V_(BYTES_VALUE); m.bytes_value.len = 0; m.bytes_value.data = nullptr;
    }
    send(s_dbirth[D_SCOPE], nullptr, ms, true);
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
    for (int d = 0; d < D_STATUS_N; d++) {
        std::vector<Metric> dm; std::vector<std::string> names;
        names.reserve(defs.size() + sdefs.size());
        for (size_t i = 0; i < defs.size(); i++)
            if (defs[i].dev == d && mirror_ok[i]) { dm.push_back(full(i, names, false)); dirty[i] = 0; }
        for (size_t k = 0; k < sdefs.size(); k++)
            if (sdefs[k].dev == d && smirror_ok[k]) { dm.push_back(str_metric(k, &names, false)); sdirty[k] = 0; }
        send(s_dbirth[d], nullptr, dm, true);
    }
    born = true;
    hal_born = false;
    if (!hpins.empty() && hal_all_ok()) hal_birth();   // else when every source reads (publisher)
    if (s_chan >= 0) scope_birth();
}

void datas()
{
    for (int d = 0; d < D_STATUS_N; d++) {
        std::vector<Metric> dm;
        for (size_t i = 0; i < defs.size(); i++) {
            if (defs[i].dev != d || !dirty[i]) continue;
            Metric m = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
            m.has_alias = 1; m.alias = i + 1;            // name and datatype were told in DBIRTH
            m.has_timestamp = 1; m.timestamp = mirror_ts[i];
            m_value(&m, defs[i].dt, mirror[i]);
            dm.push_back(m); dirty[i] = 0;
        }
        for (size_t k = 0; k < sdefs.size(); k++)
            if (sdefs[k].dev == d && sdirty[k]) { dm.push_back(str_metric(k, nullptr, false)); sdirty[k] = 0; }
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
        names.reserve(defs.size() + sdefs.size() + hpins.size());
        for (size_t i = 0; i < defs.size(); i++)
            if (mirror_ok[i] && (want.empty() || want == dev_name[defs[i].dev]))
                ms.push_back(full(i, names, true));
        for (size_t k = 0; k < sdefs.size(); k++)
            if (smirror_ok[k] && (want.empty() || want == dev_name[sdefs[k].dev]))
                ms.push_back(str_metric(k, &names, true));
        for (size_t k = 0; k < hpins.size(); k++)
            if (hpins[k].ok && (want.empty() || want == "hal"))
                ms.push_back(hal_metric(k, &names, true));
        send(r, &r, ms, false);                      // a reply, not part of the seq stream
    }
    natsMsg_Destroy(msg);
}

// ---- commands ----

// a DCMD metric -> the value for its pin, by the pin's type (hal2spb's rules); nullptr = fine
const char *cmd_value(const Metric *m, const HPin &p, hal_query_value_u &out)
{
    unsigned dt = m->has_datatype ? m->datatype : 0;
    enum { K_B, K_R, K_I, K_U } k;
    double r = 0; int64_t i = 0; uint64_t u = 0;
    switch (m->value_case) {
    case V_(BOOLEAN_VALUE):
        if (dt && dt != DT_(Boolean)) return "datatype/value mismatch";
        k = K_B; i = m->boolean_value != 0; break;
    case V_(DOUBLE_VALUE): k = K_R; r = m->double_value; break;
    case V_(FLOAT_VALUE): k = K_R; r = m->float_value; break;
    case V_(INT_VALUE):   // Int8/16/32 two's complement, UInt8/16/32
        k = K_I; i = (dt >= DT_(UInt8) && dt <= DT_(UInt32)) ? (int64_t)m->int_value : (int64_t)(int32_t)m->int_value; break;
    case V_(LONG_VALUE):
        if (dt == DT_(UInt64)) { k = K_U; u = m->long_value; } else { k = K_I; i = (int64_t)m->long_value; }
        break;
    default: return "no scalar value";
    }
    switch (p.type) {
    case HAL_BOOL:
        if (k != K_B) return "type: pin is bool";
        out.b = i != 0; return nullptr;
    case HAL_REAL:
        if (k == K_I) r = (double)i; else if (k == K_U) r = (double)u; else if (k != K_R) return "type: pin is real";
        if (!std::isfinite(r)) return "type: pin is real (finite number)";
        out.r = r; return nullptr;
    case HAL_UINT:
        if (k == K_I) { if (i < 0) return "type: pin is uint, value < 0"; u = (uint64_t)i; }
        else if (k != K_U) return "type: pin is an integer";
        out.u = u; return nullptr;
    default:
        if (k == K_U) { if (u > (uint64_t)INT64_MAX) return "type: pin is sint, value above Int64"; i = (int64_t)u; }
        else if (k != K_I) return "type: pin is an integer";
        out.s = i; return nullptr;
    }
}

// DCMD .../hal: dir=out entries by name or alias; the whole message is checked before any write
// (a lone CmdChangeRequest must not fire an old CntrlCmd whose new value was refused), then written
// in message order. No ack, no echo: the value goes out read back by the next DDATA.
void on_dcmd(natsConnection *, natsSubscription *, natsMsg *msg, void *)
{
    Payload *pl = sparkplug_b__payload__unpack(nullptr, (size_t)natsMsg_GetDataLength(msg),
                                               (const uint8_t *)natsMsg_GetData(msg));
    natsMsg_Destroy(msg);
    if (!pl) { TN_ERR("task: NATS DCMD hal refused: not a Sparkplug B payload\n"); return; }
    struct Cmd { const HPin *p; hal_query_value_u v; };
    std::vector<Cmd> cmds;
    const char *bad = nullptr; std::string who;
    for (size_t n = 0; n < pl->n_metrics && !bad; n++) {
        const Metric *m = pl->metrics[n];
        const HPin *p = nullptr;
        if (m->name) {
            who = m->name;
            for (const HPin &h : hpins) if (h.metric == who) { p = &h; break; }
        } else if (m->has_alias) {
            who = "alias " + std::to_string(m->alias);
            if (m->alias >= hal_alias0 && m->alias < hal_alias0 + hpins.size()) p = &hpins[m->alias - hal_alias0];
        }
        if (!p) { bad = "unknown metric"; break; }
        if (!p->out) { bad = "dir=in (fed by HAL, not by commands)"; break; }
        Cmd c{p, {}};
        bad = cmd_value(m, *p, c.v);
        cmds.push_back(c);
    }
    sparkplug_b__payload__free_unpacked(pl, nullptr);
    if (bad) { TN_ERR("task: NATS DCMD hal refused (%zu cmds): %s: %s\n", cmds.size(), who.c_str(), bad); return; }
    for (const Cmd &c : cmds) {
        hal_query_t q;
        memset(&q, 0, sizeof q);
        q.name = c.p->src.c_str();
        int rc;
        if (c.p->sig) { q.qtype = HAL_QTYPE_SIGNAL; q.sig.type = c.p->type; q.sig.value = c.v; rc = hal_set_s(&q, nullptr, nullptr); }
        else { q.qtype = HAL_QTYPE_PIN; q.pp.type = c.p->type; q.pp.value = c.v; rc = hal_set_p(&q, nullptr, nullptr); }
        if (rc) TN_ERR("task: NATS DCMD hal %s -> %s: %s\n", c.p->metric.c_str(), c.p->src.c_str(), hal_err(rc));
    }
}

// NCMD: "Node Control/Rebirth" only (HAL commands are DCMD of device hal)
void on_ncmd(natsConnection *, natsSubscription *, natsMsg *msg, void *)
{
    Payload *pl = sparkplug_b__payload__unpack(nullptr, (size_t)natsMsg_GetDataLength(msg),
                                               (const uint8_t *)natsMsg_GetData(msg));
    natsMsg_Destroy(msg);
    if (!pl) { TN_ERR("task: NATS NCMD refused: not a Sparkplug B payload\n"); return; }
    for (size_t n = 0; n < pl->n_metrics; n++) {
        const Metric *m = pl->metrics[n];
        if (m->name && !strcmp(m->name, "Node Control/Rebirth")) { if (m->boolean_value) need_birth = true; }
        else TN_ERR("task: NATS NCMD %s refused: the node takes Node Control/Rebirth; HAL commands are DCMD of device hal\n",
                             m->name ? m->name : "(alias)");
    }
    sparkplug_b__payload__free_unpacked(pl, nullptr);
}

void on_connected(natsConnection *, void *)
{
    rcs_print("task: NATS connected - Sparkplug births follow\n");
    need_birth = true;
}
void on_disconnected(natsConnection *, void *)
{
    rcs_print("task: NATS disconnected\n");
    std::lock_guard<std::mutex> g(mx);
    born = false;                                // no DDATA of a dead session; births on reconnect
}

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
            {
                std::lock_guard<std::mutex> sg(smx);     // the task only try_locks it
                for (size_t k = 0; k < sdefs.size(); k++)
                    if (sslot_new[k]) {
                        smirror[k].swap(sslot[k]); smirror_ts[k] = sslot_ts[k];
                        smirror_ok[k] = 1; sdirty[k] = 1; sslot_new[k] = 0;
                    }
            }
            if (!hpins.empty() && now_mono() - hal_last_read >= hal_period_s) { hal_last_read = now_mono(); hal_poll(); }
            if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) {
                bool all = true;
                for (char ok : mirror_ok) all = all && ok;
                for (char ok : smirror_ok) all = all && ok;
                if (need_birth && all) { need_birth = false; births(); }
                else if (!need_birth && born) {
                    datas();
                    if (!hpins.empty()) {
                        if (!hal_born) { if (hal_all_ok()) hal_birth(); }
                        else hal_datas();
                    }
                }
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

// The sampler FIFO -> DDATA of device scope (hal2spb's scope thread). One reader per FIFO
// (hal_stream): halsampler must not read the same channel.
void scope_main()
{
    hal_stream_t st;
    bool said = false;
    // HALFILEs run after the task: wait for the sampler component before attaching (a quiet
    // probe - hal_stream_attach logs every failure)
    // ПРЕДПОЛОЖЕНИЕ, не замер: a 1 s poll is fine for a FIFO that exists from HAL load on
    for (;;) {
        if (!running) return;
        hal_query_t q;
        memset(&q, 0, sizeof q);
        if (hal_comp_by_name("sampler", &q) == 0) {
            int r = hal_stream_attach(&st, hal_comp, SAMPLER_SHMEM_KEY + s_chan, nullptr);
            if (r >= 0) break;
            if (!said) TN_ERR("task: NATS scope: hal_stream_attach(channel %d): %s - retrying\n", s_chan, strerror(-r));
            said = true;
        } else if (!said) {
            rcs_print("task: NATS scope: no sampler component yet - waiting\n");
            said = true;
        }
        struct timespec w = {1, 0};
        nanosleep(&w, nullptr);
    }
    int ne = hal_stream_element_count(&st);
    int ncol = (int)scols.size() < ne ? (int)scols.size() : ne;
    // the FIFO holds an old tail (up to its depth) at attach: drop exactly what is queued now
    int stale = hal_stream_depth(&st), dropped_rows = 0;
    std::vector<hal_stream_data_u> row(ne > 0 ? ne : 1);
    unsigned sn;
    while (dropped_rows < stale && hal_stream_read(&st, row.data(), &sn) == 0) dropped_rows++;
    rcs_print("task: NATS scope: sampler channel %d, %d columns (%d in the conf), dropped stale tail %d\n",
              s_chan, ne, (int)scols.size(), dropped_rows);
    double dt = s_dt_ns / 1e9;
    std::vector<double> col((size_t)s_batch * (size_t)(ncol > 0 ? ncol : 1));   // column-major
    std::vector<double> sent_last(ncol > 0 ? ncol : 1);
    std::vector<char> sent_ok(ncol > 0 ? ncol : 1, 0);
    while (running) {
        int n = 0;
        while (n < s_batch && running) {
            if (!hal_stream_readable(&st)) { struct timespec w = {0, (long)(s_dt_ns > 0 ? s_dt_ns : 1000000)}; nanosleep(&w, nullptr); continue; }
            if (hal_stream_read(&st, row.data(), &sn) < 0) break;
            for (int c = 0; c < ncol; c++) {
                double x;
                switch (hal_stream_element_type(&st, c)) {
                case HAL_BOOL: x = row[c].b; break;
                case HAL_SINT: x = (double)row[c].s; break;
                case HAL_UINT: x = (double)row[c].u; break;
                default: x = row[c].f; break;
                }
                col[(size_t)c * s_batch + n] = x * scols[c].scale;
            }
            n++;
        }
        if (!n) continue;
        // time of the LAST row = now minus what is still queued behind it
        struct timespec now; clock_gettime(CLOCK_REALTIME, &now);
        double t_last = now.tv_sec + now.tv_nsec * 1e-9 - hal_stream_depth(&st) * dt;
        double t_first = t_last - (n - 1) * dt;
        uint64_t ts = (uint64_t)llround(t_first * 1000.0);
        std::vector<Metric> ms;
        for (int c = 0; c < ncol; c++) {
            // report by exception: a column constant at its last sent value is not sent
            double *x = &col[(size_t)c * s_batch];
            bool same = sent_ok[c] && x[0] == sent_last[c];
            for (int i = 1; same && i < n; i++) same = x[i] == x[0];
            if (same) continue;
            sent_ok[c] = 1; sent_last[c] = x[n - 1];
            if (s_float) { float *f = (float *)x; for (int i = 0; i < n; i++) f[i] = (float)x[i]; }
            Metric m = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
            m.has_alias = 1; m.alias = scope_alias0 + 1 + c;
            m.has_timestamp = 1; m.timestamp = ts;
            m.value_case = V_(BYTES_VALUE);
            m.bytes_value.len = (size_t)n * (s_float ? 4 : 8);   // packed LE float32/float64
            m.bytes_value.data = (uint8_t *)x;
            ms.push_back(m);
        }
        if (ms.empty()) continue;
        std::lock_guard<std::mutex> g(mx);
        // off the broker or before this session's DBIRTH: dropped (the birth declares, data follows)
        if (born && natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) send(s_ddata[D_SCOPE], nullptr, ms, true);
    }
    hal_stream_detach(&st);
}

// ---- [NATS] PINS: the hal2spb.conf line format ----

bool num(const std::string &s, double &v)
{
    char *e = nullptr;
    errno = 0;
    v = strtod(s.c_str(), &e);
    return !s.empty() && e && !*e && !errno && std::isfinite(v);
}

bool load_pins(const std::string &path)
{
    FILE *f = fopen(path.c_str(), "r");
    if (!f) {
        TN_ERR("task: [NATS] PINS %s: %s - NATS publishing refused\n", path.c_str(), strerror(errno));
        return false;
    }
    char ln[2048];
    int lineno = 0;
    bool good = true;
    std::string ignored;
    while (good && fgets(ln, sizeof ln, f)) {
        lineno++;
        std::vector<std::string> w;
        for (char *t = strtok(ln, " \t\r\n"); t; t = strtok(nullptr, " \t\r\n")) w.push_back(t);
        if (w.empty() || w[0][0] == '#') continue;
        auto bad = [&](const std::string &why) {
            TN_ERR("task: [NATS] PINS %s:%d: %s - NATS publishing refused\n", path.c_str(), lineno, why.c_str());
            good = false;
        };
        const std::string &a = w[0];
        double v = 0;
        if (a == "pin") {
            if (w.size() < 3) { bad("pin <key> <type> ..."); break; }
            HPin p{};
            p.key = w[1]; p.metric = w[1];
            for (char &c : p.metric) if (c == '.') c = '/';
            if (w[2] == "bool") p.type = HAL_BOOL;
            else if (w[2] == "sint") p.type = HAL_SINT;
            else if (w[2] == "uint") p.type = HAL_UINT;
            else if (w[2] == "real") p.type = HAL_REAL;
            else { bad("pin " + p.key + ": type '" + w[2] + "' - bool|sint|uint|real (sint/uint are 64 bit)"); break; }
            for (size_t i = 3; i < w.size() && good; i++) {
                const std::string &t = w[i];
                if (t == "dir=in" || t == "dir=out") p.out = t == "dir=out";
                else if (!t.compare(0, 4, "src=")) { p.src = t.substr(4); p.sig = false; }
                else if (!t.compare(0, 4, "sig=")) { p.src = t.substr(4); p.sig = true; }
                else if (!t.compare(0, 2, "q=")) { if (!num(t.substr(2), v)) bad(p.key + ": " + t); p.q = std::fabs(v); }
                else if (!t.compare(0, 3, "db=")) { if (!num(t.substr(3), v)) bad(p.key + ": " + t); p.db = v; }
                else if (!t.compare(0, 4, "hal=")) {}   // hal2spb's own pin name: task makes no pins
                else bad("pin " + p.key + ": '" + t + "' - dir= src= sig= q= db=");
            }
            if (!good) break;
            if (p.src.empty()) { bad("pin " + p.key + ": no src=<pin> or sig=<signal> - task reads HAL by name, it makes no pins"); break; }
            for (const HPin &o : hpins) if (o.metric == p.metric) { bad("pin " + p.key + ": a second row of the same metric"); break; }
            if (!good) break;
            if (p.type != HAL_REAL) p.q = 0;
            else if (p.q > 0 && !(p.db > 0)) p.db = p.q;   // deadband = one quantum
            if (p.type == HAL_REAL && !(p.q > 0))
                rcs_print("task: [NATS] PINS pin %s: no q= - sent unquantized (db=%g)\n", p.key.c_str(), p.db);
            hpins.push_back(p);
        } else if (a == "scope_channel" || a == "scope_batch" || a == "scope_dt_ns" || a == "spb_keepalive_s" || a == "hz") {
            if (w.size() < 2 || !num(w[1], v) || v < 0) { bad(a + ": a number >= 0"); break; }
            if (a == "hz") { if (!(v > 0)) { bad("hz > 0"); break; } hal_period_s = 1.0 / v; }
            else if (a == "scope_channel") s_chan = (int)v;
            else if (a == "scope_batch") s_batch = (int)v;
            else if (a == "scope_dt_ns") s_dt_ns = (long)v;
            else keepalive_s = v;
        } else if (a == "scope_type") {
            if (w.size() < 2 || (w[1] != "double" && w[1] != "float")) { bad("scope_type double|float"); break; }
            s_float = w[1] == "float";
        } else if (a == "scope_col") {
            size_t dot = w.size() < 2 ? std::string::npos : w[1].find('.');
            if (dot == std::string::npos) { bad("scope_col <Key.chan> [scale]"); break; }
            double sc = 1.0;
            if (w.size() > 2 && !num(w[2], sc)) { bad("scope_col " + w[1] + ": scale " + w[2]); break; }
            scols.push_back(SCol{w[1].substr(0, dot), w[1].substr(dot + 1), sc});
        } else if (a == "url" || a == "user" || a == "pass" || a == "node" || a == "spb_group"
                   || a == "spb_birth_s" || a == "format" || a == "scope_subject") {
            ignored += " " + a;                  // hal2spb's runtime keys: the INI [NATS] names them here
        } else {
            bad("unknown directive '" + a + "'");
        }
    }
    fclose(f);
    if (!good) return false;
    if (!ignored.empty())
        rcs_print("task: [NATS] PINS %s: ignored (INI [NATS] gives them):%s\n", path.c_str(), ignored.c_str());
    if (s_chan >= 0) {
        if (scols.empty() || s_batch <= 0 || s_dt_ns <= 0 || (int)scols.size() > HAL_STREAM_MAX_PINS) {
            TN_ERR("task: [NATS] PINS %s: scope_channel needs scope_col (1..%d), scope_batch > 0 and "
                            "scope_dt_ns > 0 (the sampler thread period) - NATS publishing refused\n",
                            path.c_str(), HAL_STREAM_MAX_PINS);
            return false;
        }
    }
    return true;
}

bool name_ok(const std::string &s)
{
    if (s.empty()) return false;
    for (char c : s)
        if (c == '.' || c == '/' || c == '+' || c == '#' || c == '*' || c == '>' || c <= ' ') return false;
    return true;
}

} // namespace

int taskNatsInit(const char *inifile, int joints, double task_cycle_time, int hal_comp_id)
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
        TN_ERR("task: [NATS] URL is set but GROUP/NODE is missing or not a Sparkplug id "
                        "(no . / + # * > or spaces) - NATS publishing refused, NML as before\n");
        return -1;
    }
    // the publisher's period: [NATS] PUBLISH_PERIOD, else the task cycle (one publish per task cycle at most)
    period_s = ini.findRealV("PUBLISH_PERIOD", "NATS", task_cycle_time);
    if (!(period_s > 0)) {
        TN_ERR("task: [NATS] PUBLISH_PERIOD %f is not > 0 - NATS publishing refused\n", period_s);
        return -1;
    }
    // [TRAJ] SPINDLES: LinuxCNC's own default is 1 (motion's num_spindles)
    int spindles = ini.findSIntV("SPINDLES", "TRAJ", 1);
    if (joints < 0 || joints > EMCMOT_MAX_JOINTS) joints = EMCMOT_MAX_JOINTS;
    if (spindles < 0 || spindles > EMCMOT_MAX_SPINDLES) spindles = EMCMOT_MAX_SPINDLES;

    // [NATS] PINS: devices hal and scope (the hal2spb.conf lines); relative to the INI's folder
    if (auto pins = ini.findString("PINS", "NATS")) {
        std::string path = *pins, ini_s(inifile);
        size_t sl = ini_s.rfind('/');
        if (!path.empty() && path[0] != '/' && sl != std::string::npos) path = ini_s.substr(0, sl + 1) + path;
        if (!load_pins(path)) return -1;
        if (s_chan >= 0 && hal_comp_id < 0) {
            TN_ERR("task: [NATS] PINS scope_channel without a task HAL component - NATS publishing refused\n");
            return -1;
        }
        hal_comp = hal_comp_id;
    }

    build_defs(joints, spindles);
    size_t n = defs.size();
    size_t ns = sdefs.size();
    sprev.assign(ns, std::string()); sprev_ok.assign(ns, 0);
    sslot.assign(ns, std::string()); sslot_ts.assign(ns, 0); sslot_new.assign(ns, 0);
    smirror.assign(ns, std::string()); smirror_ts.assign(ns, 0); smirror_ok.assign(ns, 0); sdirty.assign(ns, 0);
    str_alias0 = n + 1;
    hal_alias0 = str_alias0 + ns;
    scope_alias0 = hal_alias0 + hpins.size();
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
    s_ncmd = base + "NCMD." + *node;
    s_dcmd_hal = base + "DCMD." + *node + "." + dev_name[D_HAL];
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
    if (st == NATS_OK) st = natsConnection_Subscribe(&ncmd_sub, nc, s_ncmd.c_str(), on_ncmd, nullptr);
    bool has_out = false;
    for (const HPin &p : hpins) has_out = has_out || p.out;
    if (st == NATS_OK && has_out) st = natsConnection_Subscribe(&dcmd_sub, nc, s_dcmd_hal.c_str(), on_dcmd, nullptr);
    natsOptions_Destroy(opts);
    if (st != NATS_OK) {
        TN_ERR("task: NATS %s: %s - NATS publishing refused, NML as before\n",
                        url->c_str(), natsStatus_GetText(st));
        natsSubscription_Destroy(snap_sub); snap_sub = nullptr;
        natsSubscription_Destroy(ncmd_sub); ncmd_sub = nullptr;
        natsSubscription_Destroy(dcmd_sub); dcmd_sub = nullptr;
        natsConnection_Destroy(nc); nc = nullptr;
        return -1;
    }
    if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) need_birth = true;
    enabled = true;
    running = true;
    pub_thread = std::thread(publisher);
    if (s_chan >= 0) scope_thread = std::thread(scope_main);
    for (const HPin &p : hpins)
        if (p.out) rcs_print("task: NATS hal out %s (%s %s) <- %s\n", p.metric.c_str(), p.sig ? "signal" : "pin",
                             p.src.c_str(), s_dcmd_hal.c_str());
    rcs_print("task: status -> NATS %s as Sparkplug node %s/%s, %zu metrics, %zu hal, %zu scope columns, snapshot on %s\n",
              url->c_str(), group->c_str(), node->c_str(), n + ns, hpins.size(), scols.size(), s_snapshot.c_str());
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
    running = false;
    if (pub_thread.joinable()) pub_thread.join();
    if (scope_thread.joinable()) scope_thread.join();
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
    natsSubscription_Destroy(ncmd_sub); ncmd_sub = nullptr;
    natsSubscription_Destroy(dcmd_sub); dcmd_sub = nullptr;
    natsConnection_Destroy(nc); nc = nullptr;
}

#endif
