/********************************************************************
* Description: halnats.hh
*   One Sparkplug B 3.0 edge node over NATS for a HAL host - the code is one,
*   the owners are two:
*     milltask (emc/task/tasknats.cc) - a VM running LinuxCNC: adds its status
*       devices task/motion/io/config/interp as an Owner;
*     halnats (halnats_main.cc)       - a VM with HAL and no LinuxCNC (automation
*       nodes: HAL + soft-PLC): devices hal and scope only.
*   One publisher per VM: halnats refuses to run next to a LinuxCNC task (HAL
*   component iocontrol.0 present) - there the task publishes.
*
*   The node owns: the NATS connection (background connect, endless reconnect),
*   NBIRTH / NDEATH (bdSeq), NCMD "Node Control/Rebirth", the request-reply
*   snapshot lcnc.<group>.<node>.snapshot, and the devices of the PINS file:
*     hal    pins / signals read by name through the HAL query API, DDATA past
*            quantum/deadband, empty DDATA keepalive, DCMD writes dir=out entries
*     scope  the sampler FIFO, DDATA batches of Double/FloatArray columns
*   (the PINS line format is the one of appliance/hal2spb's conf - see halnats.cc).
*
*   Subjects are the NATS spelling of the Sparkplug MQTT topics
*   (spBv1//0.<group>.NBIRTH.<node> = MQTT spBv1.0/<group>/NBIRTH/<node>).
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#ifndef HALNATS_HH
#define HALNATS_HH

#include <deque>
#include <string>
#include <vector>
#include <stdint.h>

#include "sparkplug_b_payload.pb-c.h"

namespace halnats {

typedef SparkplugB__Payload__Metric Metric;
#define HN_DT(x) SPARKPLUG_B__DATA_TYPE__##x     /* Sparkplug B 3.0, section 6.4.16 */
#define HN_V(x) SPARKPLUG_B__PAYLOAD__METRIC__VALUE_##x

union Val { int64_t i; double d; };

uint64_t now_ms();                               // CLOCK_REALTIME, Sparkplug timestamps
double now_mono();
void set_value(Metric *m, uint32_t dt, Val v);   // Boolean | Int32 | Int64 | UInt64 | Double

// The metrics of one message; names live as long as the batch.
struct Batch {
    std::vector<Metric> ms;
    std::deque<std::string> names;
    // name empty = alias only (DDATA); dt 0 = datatype not told (DDATA)
    Metric &add(const std::string &name, uint64_t alias, uint32_t dt, uint64_t ts);
};

// The owner's own devices. Every call comes in the node's publisher thread (or the
// NATS callback thread for snapshot) under the node's lock - never in the owner's loop.
struct Owner {
    virtual ~Owner() {}
    virtual size_t aliases() const = 0;                      // the owner's aliases are 1..N
    virtual const std::vector<std::string> &devices() const = 0;
    virtual void pull() = 0;                                 // take what the owner's loop handed over
    virtual bool ready() const = 0;                          // every metric known: births may go
    virtual void birth(size_t dev, Batch &b) = 0;            // full metrics; marks them sent
    virtual void data(size_t dev, Batch &b) = 0;             // changes since the last birth/data
    virtual void snapshot(const std::string &want, Batch &b) = 0;   // names "<device>/<metric>"
};

typedef void (*LogFn)(bool error, const char *line);

struct Config {
    std::string url, user, pass, group, node;
    std::string pins;                            // PINS file, absolute; empty = no hal/scope devices
    double period_s = 0;                         // publisher period
    std::string tag;                             // log prefix: "task", "halnats"
    LogFn log = nullptr;                         // nullptr = stderr
};

// Connection keys from the VM environment (the agent's node-agent.env, as appliance/hal2spb):
//   SYNCTWIN_NATS_URL, SYNCTWIN_NATS_USER, SYNCTWIN_NATS_PASS, SYNCTWIN_BOX_ID -> group,
//   NODE_ID + node_suffix -> node;
// the INI [NATS] URL USER PASS GROUP NODE PINS PUBLISH_PERIOD override them (inifile may be
// nullptr). PINS relative to the INI's folder. period_s: PUBLISH_PERIOD, else default_period,
// else (<= 0) the PINS file's hz (start refuses when none gives it).
// 1 = off (no URL anywhere, said once), 0 = configured, -1 = refused (said).
int configure(Config &c, const char *inifile, const char *node_suffix, double default_period);

// 0 = publishing (connect goes on in the background); -1 = refused (said).
// hal_comp: a HAL component of the process (the sampler FIFO is attached with it).
// owner: nullptr = devices hal and scope only.
int start(const Config &c, int hal_comp, Owner *owner);
void stop();                                     // NDEATH, threads joined, connection closed

void say(bool error, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

} // namespace halnats

#endif
