/********************************************************************
* Description: tasknats.cc
*   Task status -> NATS, Sparkplug B 3.0 (protobuf); NATS is the task's only
*   command source and status sink (no NML channels). See tasknats.hh for the contract.
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
*   Node = NODE_ID + "-lcnc" (owner 23.10; aicnc spb_ncmd.lcnc_node is the one
*   home of the name): the base's halnats publishes as NODE_ID, and one
*   session is one Edge Node (Sparkplug B 3.0) - two processes never share a
*   node name. INI [NATS] NODE overrides.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#include "config.h"
#include "tasknats.hh"

#include "libnml/rcs/rcs_print.hh"

#ifndef HAVE_LIBNATS
#error "milltask needs libnats + protobuf-c (configure --with-nats): NML is gone, NATS is the task's only command source and status sink"
#else

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <limits.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <inifile.hh>
#include "halnats.hh"
#include <rtapi_string.h>                        // rtapi_strlcpy()
#include "nml_intf/emc.hh"
#include "nml_intf/emc_nml.hh"
#include "motion/usrmotintf.h"            // usrmotReadEmcmotConfig(): num_dio / num_aio
#include "motion/motion.h"
#include "tp/tp_types.h"                  // TP_VEL_EPSILON

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

// node metric cell/moving (NBIRTH + NDATA of the node, by exception): the cell is in a process of motion -
// a program's or MDI's coordinated move, a jog, a homing. The task thread folds it each cycle into one word
// (ts_ms << 1 | moving; 0 = not computed yet), the publisher sends it only when the value turns.
std::atomic<uint64_t> moving_word{0};
int moving_sent = -1;                            // publisher side (under the node's lock): -1 = none sent
int moving_joints = 0;                           // the published joints (taskNatsInit)

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

void build_defs(int joints, int spindles, int dio, int aio)
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
    // motion's digital / analog IO (M62-M68, motion.digital-in-NN / -out-NN, analog-in-NN / -out-NN):
    // the motmod num_dio / num_aio there are, not EMCMOT_MAX_* - an IO that does not exist is not a zero
    for (int n = 0; n < dio; n++) {
        AB(D_MOTION, "synch_di/" + std::to_string(n), s->motion.synch_di[n]);
        AB(D_MOTION, "synch_do/" + std::to_string(n), s->motion.synch_do[n]);
    }
    for (int n = 0; n < aio; n++) {
        AR(D_MOTION, "analog_input/" + std::to_string(n), s->motion.analog_input[n]);
        AR(D_MOTION, "analog_output/" + std::to_string(n), s->motion.analog_output[n]);
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

// ---- commands: DCMD of task / motion -> the task loop (the only command source) ----
//
// The NATS delivery threads (one per device) parse a DCMD into EMC messages (the emc.hh classes the
// planner takes) and push it into a lock-free ring (producers serialized by cmx, the task loop pops
// without a lock). The task takes ONE message per cycle and plans it (emcTaskPlan: every state /
// mode / interlock check applies; nothing is bypassed).
// Serial: the sender's cmd/serial must grow (a repeat or an old one is refused - a resend is
// harmless); inside the task the message carries a NEGATIVE serial of its own, never 0 (the null
// command the task starts with), so the planner's "new command" test (serial != echo) is exact.
// One at a time: the next queued command is planned only after the final answer (DONE / ERROR) of
// the one in hand - every cmd/serial gets exactly one final answer, and an MDI line holds the
// queue until its move is done.
// Immediate lane (the task's own "immediate commands": emcTaskPlan issues them at once, beside a
// running program - see immediate_type): they go to a ring of their own, are planned without
// waiting for the final answer of the command in hand and answer for themselves in the cycle they
// are planned (the plan's result, emcTaskPlan's return); the command in hand keeps running and gets
// its own final answer from the task's status - program/run paused by program/pause stays EXEC and
// ends DONE when the program does. Between the queue and the immediate lane cmd/serial order holds
// while nothing is in hand; while a command is in hand, immediate ones overtake the queue.
// Priority lane (as Abort above all in PLCopen / ISA-88): state ESTOP and abort go to a ring of
// their own that the task takes first - ahead of the queue and of the rest of the command in hand.
// What they overtake is aborted, not left hanging: the command in hand and the commands queued
// before them (lower cmd/serial) are answered ERROR "aborted by ...". A full ring refuses the
// command with an answer ERROR "command queue full" - never silently, and a resend of the same
// cmd/serial is accepted (it was not taken).
// Queue capacity: [TASK] MDI_QUEUED_COMMANDS - the very variable of the task's MDI queue
// (max_mdi_queued_commands, default MAX_MDI_QUEUE 10), handed in by taskNatsInit.

struct NCmd {
    int64_t serial = 0;                          // the sender's cmd/serial
    bool prio = false;                           // state ESTOP | abort: the priority lane
    bool imm = false;                            // immediate_type: the immediate lane
    std::string verb;                            // the verb, for the answers of what it aborts
    std::vector<std::unique_ptr<RCS_CMD_MSG>> msgs;  // program/open = CLOSE + OPEN
    std::string refused;                         // non-empty: answered RCS_ERROR, nothing planned
};

// SPSC under cmx: producers (NATS delivery threads) serialized by cmx, the task thread pops lock-free
struct CmdRing {
    std::vector<NCmd *> slot;
    std::atomic<size_t> head{0}, tail{0};
    CmdRing() = default;
    explicit CmdRing(size_t n) : slot(n) {}
    bool full() const { return head.load(std::memory_order_relaxed) - tail.load(std::memory_order_acquire) >= slot.size(); }
    void push(NCmd *c) { size_t h = head.load(std::memory_order_relaxed); slot[h % slot.size()] = c; head.store(h + 1, std::memory_order_release); }
    NCmd *front() const { size_t t = tail.load(std::memory_order_relaxed); return t == head.load(std::memory_order_acquire) ? nullptr : slot[t % slot.size()]; }
    void drop_front() { tail.store(tail.load(std::memory_order_relaxed) + 1, std::memory_order_release); }
};
CmdRing cring;                                   // sized by taskNatsInit: [TASK] MDI_QUEUED_COMMANDS
CmdRing iring;                                   // immediate lane, sized as cring
CmdRing pring(8);                                // ПРЕДПОЛОЖЕНИЕ, не замер: 8 E-STOP / abort between two task cycles
std::mutex cmx;                                  // producers only (NATS delivery threads)
int64_t c_last_serial = INT64_MIN;               // under cmx: the last accepted cmd/serial
bool c_enabled = false;

// task thread: the command being planned / answered
std::unique_ptr<NCmd> c_cur;
size_t c_next = 0;                               // next message of c_cur to hand out
int c_internal = 0;                              // the negative serial counter
int c_cur_internal = 0;                          // the internal serial of c_cur's last handed message
int c_rep_state = -1;                            // the last answered state of c_cur (-1 = none yet)
int c_status_seen = -1;                          // emcStatus->status at the end of the last cycle
std::string c_err;                               // the operator errors raised while c_cur runs
bool c_sup = false;                              // c_cur overtaken by an immediate command: its answer from the task's status
std::unique_ptr<NCmd> c_imm;                     // the immediate command last handed (alive while it is emcCommand)
bool c_imm_open = false;                         // c_imm handed, its plan's result not yet answered
std::string c_imm_err;                           // the operator errors raised while c_imm is planned

// ---- events: answers and operator messages -> device task (lock-free ring, producer lock emx) ----
enum EvKind { EV_CMD, EV_ERROR, EV_TEXT, EV_DISPLAY };
struct Event {
    uint8_t kind;
    int32_t state;                               // EV_CMD: RCS_STATUS (DONE 1, EXEC 2, ERROR 3)
    int64_t serial;                              // EV_CMD: the sender's cmd/serial
    uint64_t ts_ms;
    char text[LINELEN];                          // EV_CMD: the error, else the message
};
const size_t EV_RING = 256;                      // ПРЕДПОЛОЖЕНИЕ, не замер: 256 events between two publishes - to measure under load
Event ering[EV_RING];
std::atomic<size_t> e_head{0}, e_tail{0};
std::mutex emx;                                  // producers (task thread; serialized for safety)
std::atomic<uint64_t> e_dropped{0};
uint64_t e_dropped_said = 0;

// publisher side (under the node's lock): events taken by pull, sent by data, last values for births
std::vector<Event> e_pending;
Event e_last_cmd{}, e_last_msg[3]{};
uint64_t ev_alias0 = 0;                          // aliases: status, strings, then the 6 event metrics
enum { EA_SERIAL, EA_STATE, EA_CMDERR, EA_ERROR, EA_TEXT, EA_DISPLAY, EA_N };
const char *const ev_name[EA_N] = {"cmd/serial", "cmd/state", "cmd/error",
                                   "operator/error", "operator/text", "operator/display"};

void push_event(uint8_t kind, int64_t serial, int32_t state, const char *text)
{
    std::lock_guard<std::mutex> g(emx);          // uncontended: the task thread is the producer
    size_t h = e_head.load(std::memory_order_relaxed);
    if (h - e_tail.load(std::memory_order_acquire) >= EV_RING) { e_dropped.fetch_add(1); return; }
    Event &e = ering[h % EV_RING];
    e.kind = kind; e.serial = serial; e.state = state; e.ts_ms = halnats::now_ms();
    snprintf(e.text, sizeof e.text, "%s", text ? text : "");
    e_head.store(h + 1, std::memory_order_release);
}

bool metric_int(const Metric *m, int64_t &v)
{
    if (m->value_case == HN_V(INT_VALUE)) {
        bool u = m->has_datatype && m->datatype >= DT_(UInt8) && m->datatype <= DT_(UInt32);
        v = u ? (int64_t)m->int_value : (int64_t)(int32_t)m->int_value; return true;
    }
    if (m->value_case == HN_V(LONG_VALUE)) { v = (int64_t)m->long_value; return true; }
    return false;
}
bool metric_real(const Metric *m, double &v)
{
    int64_t i;
    if (m->value_case == HN_V(DOUBLE_VALUE)) { v = m->double_value; return std::isfinite(v); }
    if (m->value_case == HN_V(FLOAT_VALUE)) { v = m->float_value; return std::isfinite(v); }
    if (metric_int(m, i)) { v = (double)i; return true; }
    return false;
}
bool metric_true(const Metric *m) { return m->value_case == HN_V(BOOLEAN_VALUE) && m->boolean_value; }

// The task's immediate commands that may come beside a running program - from the task itself:
// emcTaskPlan (emctaskmain.cc) issues these by emcTaskIssueCommand at once in all three states of a
// running program (ON, AUTO, interpState READING / WAITING / PAUSED; PLAN_STEP is handled there in
// all three), never through interp_list ("Immediate commands, i.e., commands sent from calls to
// emcTaskIssueCommand() in emcTaskPlan() directly" - emcTaskCheckPreconditions). Of that list
// SET_MODE, SET_STATE and ABORT stay out: they abort the running program (emcTaskSetMode MDI ->
// emcTaskAbort, emctask.cc; state OFF / ESTOP; abort) - state ESTOP and abort have the priority lane.
bool immediate_type(NMLTYPE t)
{
    switch (t) {
    case EMC_TASK_PLAN_PAUSE_TYPE:
    case EMC_TASK_PLAN_RESUME_TYPE:
    case EMC_TASK_PLAN_STEP_TYPE:
    case EMC_TRAJ_SET_SCALE_TYPE:
    case EMC_TRAJ_SET_RAPID_SCALE_TYPE:
    case EMC_TRAJ_SET_MAX_VELOCITY_TYPE:
    case EMC_TRAJ_SET_SPINDLE_SCALE_TYPE:
    case EMC_JOINT_UNHOME_TYPE:
        return true;
    default:
        return false;
    }
}

// One DCMD -> NCmd. Device task: state | mode | mdi | program/open | program/run | program/pause |
// program/resume | program/step | abort; device motion: home | unhome | teleop | jog/stop |
// jog/continuous | jog/increment (+ jog/joint, jog/velocity, jog/distance) | feedrate | rapidrate |
// spindleoverride (+ spindleoverride/spindle) | maxvel. Exactly one verb + cmd/serial.
// The values and the checks are the Python linuxcnc.command's (emcmodule.cc), nothing more.
std::string parse_cmd(size_t dev, const SparkplugB__Payload *pl, NCmd &c, bool &has_serial)
{
    const Metric *verb = nullptr, *jj = nullptr, *jv = nullptr, *jd = nullptr, *sn = nullptr;
    has_serial = false;
    for (size_t n = 0; n < pl->n_metrics; n++) {
        const Metric *m = pl->metrics[n];
        if (!m->name) return "metrics by name only (alias without birth of commands)";
        std::string nm = m->name;
        if (nm == "cmd/serial") { if (!metric_int(m, c.serial)) return "cmd/serial: integer"; has_serial = true; }
        else if (dev == D_MOTION && nm == "jog/joint") jj = m;
        else if (dev == D_MOTION && nm == "jog/velocity") jv = m;
        else if (dev == D_MOTION && nm == "jog/distance") jd = m;
        else if (dev == D_MOTION && nm == "spindleoverride/spindle") sn = m;
        else if (verb) return "one command per DCMD (" + std::string(verb->name) + " and " + nm + ")";
        else verb = m;
    }
    if (!has_serial) return "no cmd/serial";
    if (!verb) return "no command";
    std::string v = verb->name;
    c.verb = v;
    int64_t i = 0;
    const char *str = verb->value_case == HN_V(STRING_VALUE) ? verb->string_value : nullptr;
    auto put = [&](RCS_CMD_MSG *m) { c.msgs.emplace_back(m); };
    if (dev == D_TASK) {
        if (v == "state") {
            if (!metric_int(verb, i) || i < (int)EMC_TASK_STATE::ESTOP || i > (int)EMC_TASK_STATE::ON)
                return "state: 1 ESTOP | 2 ESTOP_RESET | 3 OFF | 4 ON";
            auto *m = new EMC_TASK_SET_STATE; m->state = (EMC_TASK_STATE)i; put(m);
            c.prio = i == (int)EMC_TASK_STATE::ESTOP;
        } else if (v == "mode") {
            if (!metric_int(verb, i) || i < (int)EMC_TASK_MODE::MANUAL || i > (int)EMC_TASK_MODE::MDI)
                return "mode: 1 MANUAL | 2 AUTO | 3 MDI";
            auto *m = new EMC_TASK_SET_MODE; m->mode = (EMC_TASK_MODE)i; put(m);
        } else if (v == "mdi") {
            auto *m = new EMC_TASK_PLAN_EXECUTE;
            if (!str || strlen(str) > sizeof(m->command) - 1) { delete m; return "mdi: a string under " + std::to_string(sizeof(m->command)) + " chars"; }
            rtapi_strlcpy(m->command, str, sizeof(m->command)); put(m);
        } else if (v == "program/open") {
            auto *m = new EMC_TASK_PLAN_OPEN;
            if (!str || !*str || strlen(str) > sizeof(m->file) - 1) { delete m; return "program/open: a file path under " + std::to_string(sizeof(m->file)) + " chars"; }
            rtapi_strlcpy(m->file, str, sizeof(m->file));
            m->remote_buffersize = 0; m->remote_filesize = 0;      // the file is local to the task
            put(new EMC_TASK_PLAN_CLOSE); put(m);
        } else if (v == "program/run") {
            if (!metric_int(verb, i) || i < 0 || i > INT_MAX) return "program/run: the start line (0 = from the top)";
            auto *m = new EMC_TASK_PLAN_RUN; m->line = (int)i; put(m);
        } else if (v == "program/pause") { if (!metric_true(verb)) return v + ": Boolean true"; put(new EMC_TASK_PLAN_PAUSE); }
        else if (v == "program/resume") { if (!metric_true(verb)) return v + ": Boolean true"; put(new EMC_TASK_PLAN_RESUME); }
        else if (v == "program/step") { if (!metric_true(verb)) return v + ": Boolean true"; put(new EMC_TASK_PLAN_STEP); }
        else if (v == "abort") { if (!metric_true(verb)) return v + ": Boolean true"; put(new EMC_TASK_ABORT); c.prio = true; }
        else return "unknown command of device task: " + v;
        return "";
    }
    // motion
    if (v == "home" || v == "unhome") {
        if (!metric_int(verb, i) || i < -1 || i >= EMCMOT_MAX_JOINTS) return v + ": joint index, -1 = all";
        if (v == "home") { auto *m = new EMC_JOINT_HOME; m->joint = (int)i; put(m); }
        else { auto *m = new EMC_JOINT_UNHOME; m->joint = (int)i; put(m); }
    } else if (v == "teleop") {
        if (verb->value_case != HN_V(BOOLEAN_VALUE)) return "teleop: Boolean";
        auto *m = new EMC_TRAJ_SET_TELEOP_ENABLE; m->enable = verb->boolean_value ? 1 : 0; put(m);
    } else if (v == "jog/stop" || v == "jog/continuous" || v == "jog/increment") {
        if (!metric_int(verb, i) || i < 0 || i >= EMCMOT_MAX_JOINTS) return v + ": joint or axis index";
        if (!jj || jj->value_case != HN_V(BOOLEAN_VALUE)) return v + ": jog/joint Boolean (true = joint, false = axis)";
        int jm = jj->boolean_value ? 1 : 0;
        double vel = 0, inc = 0;
        if (v != "jog/stop" && (!jv || !metric_real(jv, vel))) return v + ": jog/velocity";
        if (v == "jog/increment" && (!jd || !metric_real(jd, inc))) return v + ": jog/distance";
        if (v == "jog/stop") { auto *m = new EMC_JOG_STOP; m->joint_or_axis = (int)i; m->jjogmode = jm; put(m); }
        else if (v == "jog/continuous") { auto *m = new EMC_JOG_CONT; m->joint_or_axis = (int)i; m->vel = vel; m->jjogmode = jm; put(m); }
        else { auto *m = new EMC_JOG_INCR; m->joint_or_axis = (int)i; m->vel = vel; m->incr = inc; m->jjogmode = jm; put(m); }
    } else if (v == "feedrate" || v == "rapidrate" || v == "maxvel") {
        // a fraction (1.0 = 100 %) / maxvel in machine units per second, as linuxcnc.command takes them
        double d = 0;
        if (!metric_real(verb, d)) return v + ": a finite number";
        if (v == "feedrate") { auto *m = new EMC_TRAJ_SET_SCALE; m->scale = d; put(m); }
        else if (v == "rapidrate") { auto *m = new EMC_TRAJ_SET_RAPID_SCALE; m->scale = d; put(m); }
        else { auto *m = new EMC_TRAJ_SET_MAX_VELOCITY; m->velocity = d; put(m); }
    } else if (v == "spindleoverride") {
        double d = 0;
        int64_t sp = 0;                          // linuxcnc.command's default: spindle 0
        if (!metric_real(verb, d)) return v + ": a finite number (1.0 = 100 %)";
        if (sn && (!metric_int(sn, sp) || sp < 0 || sp >= EMCMOT_MAX_SPINDLES))
            return v + ": spindleoverride/spindle 0.." + std::to_string(EMCMOT_MAX_SPINDLES - 1);
        auto *m = new EMC_TRAJ_SET_SPINDLE_SCALE; m->scale = d; m->spindle = (int)sp; put(m);
    } else return "unknown command of device motion: " + v;
    return "";
}

void on_command(size_t dev, const SparkplugB__Payload *pl)
{
    std::unique_ptr<NCmd> c(new NCmd);
    bool has_serial = false;
    std::string bad = parse_cmd(dev, pl, *c, has_serial);
    c->imm = bad.empty() && c->msgs.size() == 1 && immediate_type(c->msgs[0]->_type);
    std::lock_guard<std::mutex> g(cmx);
    if (bad.empty() && c->serial <= c_last_serial)
        bad = "cmd/serial " + std::to_string(c->serial) + " not after " + std::to_string(c_last_serial) + " (repeat ignored)";
    if (!bad.empty()) {
        c->msgs.clear();
        c->refused = bad;
        rcs_print_error("task: NATS DCMD %s refused: %s\n", dev_names[dev].c_str(), bad.c_str());
        if (!has_serial) return;                 // nothing to answer to
    }
    CmdRing &r = c->prio ? pring : c->imm ? iring : cring;
    if (r.full()) {                              // refused with an answer; c_last_serial untouched: a resend is accepted
        std::string why = std::string(c->prio ? "priority " : c->imm ? "immediate " : "") + "command queue full: "
                        + std::to_string(r.slot.size()) + " commands not yet taken by the task"
                        + (c->prio ? "" : " ([TASK] MDI_QUEUED_COMMANDS)");
        rcs_print_error("task: NATS DCMD %s cmd/serial %lld refused: %s\n", dev_names[dev].c_str(), (long long)c->serial, why.c_str());
        push_event(EV_CMD, c->serial, (int)RCS_STATUS::ERROR, why.c_str());
        return;
    }
    if (c->refused.empty()) c_last_serial = c->serial;
    r.push(c.release());
}

// task thread, end of cycle: the answer of the command in hand, on change only
void answer_cycle(const EMC_STAT *stat)
{
    c_status_seen = (int)stat->status;
    if (!c_cur || !c_cur->refused.empty()) return;
    // DONE / ERROR is the command's last answer: a later error of the machine is not this command's
    if (c_rep_state == (int)RCS_STATUS::DONE || c_rep_state == (int)RCS_STATUS::ERROR) return;
    int st;
    if (c_sup) st = (int)stat->status;          // overtaken: the task's status is the command in hand's (the immediate one's errors are its own)
    else if (stat->echo_serial_number != c_cur_internal) st = c_rep_state;   // not echoed yet / superseded: the last answer stands
    else if (c_next < c_cur->msgs.size()) st = stat->status == RCS_STATUS::ERROR ? (int)RCS_STATUS::ERROR : (int)RCS_STATUS::EXEC;
    else st = (int)stat->status;
    if (st == c_rep_state || st < 0) return;
    c_rep_state = st;
    push_event(EV_CMD, c_cur->serial, st, st == (int)RCS_STATUS::ERROR ? c_err.c_str() : "");
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

void ev_metric(Batch &b, int k, const Event &e, bool named, bool prefixed)
{
    std::string n = named ? (prefixed ? std::string("task/") + ev_name[k] : std::string(ev_name[k])) : std::string();
    uint32_t dt = k == EA_SERIAL ? DT_(Int64) : k == EA_STATE ? DT_(Int32) : DT_(String);
    Metric &m = b.add(n, ev_alias0 + k, named ? dt : 0, e.ts_ms);
    if (k == EA_SERIAL) halnats::set_value(&m, dt, I(e.serial));
    else if (k == EA_STATE) halnats::set_value(&m, dt, I(e.state));
    else { m.value_case = HN_V(STRING_VALUE); m.string_value = (char *)e.text; }
}
void moving_metric(Batch &b, uint64_t w)
{
    Metric &m = b.add("cell/moving", 0, DT_(Boolean), w >> 1);
    halnats::set_value(&m, DT_(Boolean), I((int64_t)(w & 1)));
    moving_sent = (int)(w & 1);
}

void ev_last_all(Batch &b, bool named, bool prefixed)
{
    ev_metric(b, EA_SERIAL, e_last_cmd, named, prefixed);
    ev_metric(b, EA_STATE, e_last_cmd, named, prefixed);
    ev_metric(b, EA_CMDERR, e_last_cmd, named, prefixed);
    for (int k = 0; k < 3; k++) ev_metric(b, EA_ERROR + k, e_last_msg[k], named, prefixed);
}

struct TaskStatus : halnats::Owner {
    size_t aliases() const override { return defs.size() + sdefs.size() + EA_N; }
    bool takes_commands(size_t d) const override { return d == D_TASK || d == D_MOTION; }
    void command(size_t d, const SparkplugB__Payload *pl) override { on_command(d, pl); }
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
        take_events();                           // the last batch is sent: its events become the last values
        size_t et = e_tail.load(std::memory_order_relaxed), eh = e_head.load(std::memory_order_acquire);
        for (; et != eh; et++) e_pending.push_back(ering[et % EV_RING]);
        e_tail.store(et, std::memory_order_release);
        uint64_t edr = e_dropped.load(std::memory_order_relaxed);
        if (edr != e_dropped_said) {
            rcs_print("task: NATS event ring full - %llu answers/messages dropped so far\n", (unsigned long long)edr);
            e_dropped_said = edr;
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
        return moving_word.load(std::memory_order_acquire) != 0;
    }
    void birth(size_t d, Batch &b) override
    {
        for (size_t i = 0; i < defs.size(); i++)
            if ((size_t)defs[i].dev == d && mirror_ok[i]) { status_metric(b, i, true, false); dirty[i] = 0; }
        for (size_t k = 0; k < sdefs.size(); k++)
            if ((size_t)sdefs[k].dev == d && smirror_ok[k]) { str_metric(b, k, true, false); sdirty[k] = 0; }
        if (d == D_TASK) { take_events(); ev_last_all(b, true, false); }
    }
    void node_birth(Batch &b) override
    {
        uint64_t w = moving_word.load(std::memory_order_acquire);
        if (w) moving_metric(b, w);
    }
    void node_data(Batch &b) override
    {
        uint64_t w = moving_word.load(std::memory_order_acquire);
        if (w && (int)(w & 1) != moving_sent) moving_metric(b, w);
    }
    // fold the events into the last values (a birth carries those) and forget them
    void take_events()
    {
        for (const Event &e : e_pending) {
            if (e.kind == EV_CMD) e_last_cmd = e; else e_last_msg[e.kind - EV_ERROR] = e;
        }
        e_pending.clear();
    }
    void data(size_t d, Batch &b) override
    {
        for (size_t i = 0; i < defs.size(); i++)
            if ((size_t)defs[i].dev == d && dirty[i]) { status_metric(b, i, false, false); dirty[i] = 0; }
        for (size_t k = 0; k < sdefs.size(); k++)
            if ((size_t)sdefs[k].dev == d && sdirty[k]) { str_metric(b, k, false, false); sdirty[k] = 0; }
        if (d == D_TASK) {
            // every event is its own metric value with its own timestamp (repeated metrics in one DDATA)
            for (const Event &e : e_pending) {
                if (e.kind == EV_CMD) {
                    ev_metric(b, EA_SERIAL, e, false, false); ev_metric(b, EA_STATE, e, false, false);
                    ev_metric(b, EA_CMDERR, e, false, false);
                } else ev_metric(b, EA_ERROR + (e.kind - EV_ERROR), e, false, false);
            }
            // the batch points into e_pending's texts: they stay until the next pull (the batch is sent first)
        }
    }
    void snapshot(const std::string &want, Batch &b) override
    {
        for (size_t i = 0; i < defs.size(); i++)
            if (mirror_ok[i] && (want.empty() || want == dev_names[defs[i].dev])) status_metric(b, i, true, true);
        for (size_t k = 0; k < sdefs.size(); k++)
            if (smirror_ok[k] && (want.empty() || want == dev_names[sdefs[k].dev])) str_metric(b, k, true, true);
        if (want.empty() || want == "task") ev_last_all(b, true, true);
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

// the task's own Edge Node next to the base's NODE_ID (aicnc spb_ncmd.lcnc_node)
static const char *const TASK_NODE_SUFFIX = "-lcnc";

int taskNatsInit(const char *inifile, int joints, double task_cycle_time, int hal_comp_id, int cmd_queue)
{
    halnats::Config c;
    c.tag = "task";
    c.log = log_line;
    // the publisher's period: [NATS] PUBLISH_PERIOD, else the task cycle (one publish per task cycle at most)
    int r = halnats::configure(c, inifile, TASK_NODE_SUFFIX, task_cycle_time);
    if (r > 0) {
        rcs_print_error("task: no NATS URL ([NATS] URL or SYNCTWIN_NATS_URL) - NATS is the only command "
                        "source and status sink (no NML), the task cannot run without it\n");
        return -1;
    }
    if (r) return -1;

    // [TRAJ] SPINDLES: LinuxCNC's own default is 1 (motion's num_spindles)
    linuxcnc::IniFile ini(inifile);
    int spindles = ini ? ini.findSIntV("SPINDLES", "TRAJ", 1) : 1;
    if (joints < 0 || joints > EMCMOT_MAX_JOINTS) joints = EMCMOT_MAX_JOINTS;
    if (spindles < 0 || spindles > EMCMOT_MAX_SPINDLES) spindles = EMCMOT_MAX_SPINDLES;

    // motmod num_dio / num_aio: from motion's config (shared memory is up after emctask_startup)
    int dio = 0, aio = 0;
    emcmot_config_t mc;
    if (0 == usrmotReadEmcmotConfig(&mc)) {
        dio = mc.numDIO < 0 ? 0 : mc.numDIO > EMCMOT_MAX_DIO ? EMCMOT_MAX_DIO : mc.numDIO;
        aio = mc.numAIO < 0 ? 0 : mc.numAIO > EMCMOT_MAX_AIO ? EMCMOT_MAX_AIO : mc.numAIO;
    } else
        log_line(true, "task: NATS - motion's config not read, motion synch_di/do and analog_input/output not published\n");

    build_defs(joints, spindles, dio, aio);
    moving_joints = joints;
    size_t n = defs.size();
    size_t ns = sdefs.size();
    sprev.assign(ns, std::string()); sprev_ok.assign(ns, 0);
    sslot.assign(ns, std::string()); sslot_ts.assign(ns, 0); sslot_new.assign(ns, 0);
    smirror.assign(ns, std::string()); smirror_ts.assign(ns, 0); smirror_ok.assign(ns, 0); sdirty.assign(ns, 0);
    str_alias0 = n + 1;
    ev_alias0 = str_alias0 + ns;
    prev.assign(n, Val{}); prev_ok.assign(n, 0);
    mirror.assign(n, Val{}); mirror_ts.assign(n, 0); mirror_ok.assign(n, 0); dirty.assign(n, 0);
    // ring capacity: four full snapshots, rounded up to a power of two
    // (ПРЕДПОЛОЖЕНИЕ, не замер: 4 snapshots cover a publisher lagging 4 periods)
    size_t cap = 1;
    while (cap < 4 * n) cap <<= 1;
    ring.assign(cap, Change{}); ring_mask = cap - 1;

    // command queue: [TASK] MDI_QUEUED_COMMANDS (cmd_queue); 0 = nothing waits behind the one in hand
    // but the hand-over slot itself, so one slot at least
    cring.slot.assign(cmd_queue > 0 ? (size_t)cmd_queue : 1, nullptr);
    iring.slot.assign(cring.slot.size(), nullptr);

    if (halnats::start(c, hal_comp_id, &owner)) return -1;
    enabled = true;
    c_enabled = true;
    return 0;
}

// Task main loop, every cycle: the next NATS message to plan (emcCommand = it), or nullptr.
// *immediate: it is an immediate command - the caller hands emcTaskPlan's result to taskNatsPlanned.
RCS_CMD_MSG *taskNatsCommand(bool *immediate)
{
    *immediate = false;
    if (!c_enabled) return nullptr;
    if (c_cur && c_next < c_cur->msgs.size() && c_rep_state == (int)RCS_STATUS::ERROR)
        c_next = c_cur->msgs.size();             // program/open: CLOSE failed - OPEN is not sent
    bool c_final = c_rep_state == (int)RCS_STATUS::DONE || c_rep_state == (int)RCS_STATUS::ERROR;
    if (NCmd *p = pring.front()) {               // E-STOP / abort: ahead of everything
        pring.drop_front();
        std::unique_ptr<NCmd> n(p);
        if (!n->refused.empty()) {
            push_event(EV_CMD, n->serial, (int)RCS_STATUS::ERROR, n->refused.c_str());
            return nullptr;
        }
        std::string why = "aborted by " + n->verb + " (cmd/serial " + std::to_string(n->serial) + ")";
        if (c_cur && c_cur->refused.empty() && !c_final)
            push_event(EV_CMD, c_cur->serial, (int)RCS_STATUS::ERROR, why.c_str());
        for (CmdRing *r : {&cring, &iring})      // queued before it (lower serial): aborted, never run after it
            while (NCmd *q = r->front()) {
                if (q->serial >= n->serial) break;
                r->drop_front();
                push_event(EV_CMD, q->serial, (int)RCS_STATUS::ERROR, q->refused.empty() ? why.c_str() : q->refused.c_str());
                delete q;
            }
        c_cur = std::move(n);
        c_next = 0; c_rep_state = -1; c_err.clear(); c_sup = false;
    } else if (!c_cur || c_next >= c_cur->msgs.size()) {
        // one at a time, in cmd/serial order: the next is taken only after the final answer of the one
        // in hand; an immediate command is taken at once (while one is in hand it overtakes the queue)
        bool busy = c_cur && !c_final;
        NCmd *q = cring.front(), *i = iring.front();
        bool take_imm = i && (busy || !q || i->serial < q->serial);
        if (!take_imm && (busy || !q)) return nullptr;
        CmdRing &r = take_imm ? iring : cring;
        r.drop_front();
        std::unique_ptr<NCmd> n(take_imm ? i : q);
        if (!n->refused.empty()) {               // answered here; c_cur / c_imm (maybe still emcCommand) stay alive
            push_event(EV_CMD, n->serial, (int)RCS_STATUS::ERROR, n->refused.c_str());
            return nullptr;
        }
        if (take_imm) {                          // c_cur stays in hand; its answer now from the task's status
            if (busy) c_sup = true;
            c_imm = std::move(n);                // the old c_imm is replaced as emcCommand by the caller now
            c_imm_open = true; c_imm_err.clear();
            if (c_internal <= INT_MIN + 1) c_internal = 0;
            RCS_CMD_MSG *m = c_imm->msgs[0].get();
            m->serial_number = --c_internal;
            *immediate = true;
            return m;
        }
        c_cur = std::move(n);                    // the old message is replaced as emcCommand by the caller now
        c_next = 0; c_rep_state = -1; c_err.clear(); c_sup = false;
    }
    RCS_CMD_MSG *m = c_cur->msgs[c_next++].get();
    if (c_internal <= INT_MIN + 1) c_internal = 0;
    m->serial_number = c_cur_internal = --c_internal;
    return m;
}

// Task main loop, right after emcTaskPlan planned an immediate command: its answer (0 = DONE).
// Its result stays out of the task's taskPlanError - that status is the command in hand's.
void taskNatsPlanned(int plan_rc)
{
    if (!c_imm || !c_imm_open) return;
    c_imm_open = false;
    if (plan_rc == 0) push_event(EV_CMD, c_imm->serial, (int)RCS_STATUS::DONE, "");
    else push_event(EV_CMD, c_imm->serial, (int)RCS_STATUS::ERROR,
                    c_imm_err.empty() ? "refused by the task's plan" : c_imm_err.c_str());
}

// emcOperatorError / Text / Display (kind 0 / 1 / 2) of the task process, next to the NML error channel
void taskNatsMessage(int kind, const char *text)
{
    if (!c_enabled || kind < 0 || kind > 2) return;
    push_event((uint8_t)(EV_ERROR + kind), 0, 0, text);
    if (kind != 0) return;
    // the text of a failing NATS command's answer: the immediate one being planned, else the one in hand
    std::string *err = c_imm_open ? &c_imm_err
                     : (c_cur && c_rep_state != (int)RCS_STATUS::DONE && c_rep_state != (int)RCS_STATUS::ERROR) ? &c_err
                     : nullptr;
    if (err && err->size() < LINELEN) {
        if (!err->empty()) *err += "; ";
        *err += text;
    }
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
    // cell/moving: motion's own "not in position" (control.c: a free_tp active in jog/home, the coord tp
    // not done), a speed above the planner's own zero (tp_types.h TP_VEL_EPSILON), or a joint homing
    // (a homing sequence also stands still between its moves)
    bool mv = !stat->motion.traj.inpos || std::fabs(stat->motion.traj.current_vel) > TP_VEL_EPSILON;
    for (int j = 0; !mv && j < moving_joints; j++) mv = stat->motion.joint[j].homing;
    uint64_t w = moving_word.load(std::memory_order_relaxed);
    if (!w || (int)(w & 1) != (int)mv) moving_word.store(ts << 1 | (mv ? 1 : 0), std::memory_order_release);
    answer_cycle(stat);
}

void taskNatsShutdown(void)
{
    if (!enabled) return;
    enabled = false;
    c_enabled = false;
    halnats::stop();
}

#endif
