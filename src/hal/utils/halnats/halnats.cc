/********************************************************************
* Description: halnats.cc
*   The Sparkplug B node of a HAL host over NATS - see halnats.hh.
*
*   PINS file (the appliance/hal2spb conf line format, which this replaces on the VM):
*     pin <key> bool|sint|uint|real [dir=in|out] src=<pin> | sig=<signal> [q=<quantum>] [db=<deadband>]
*            Device hal. Read by name through the HAL query API (hal_get_p / hal_get_s, the
*            link resolved at every read, so a `net` made later is followed); the node makes
*            no pins of its own, a source not there yet is waited for and the DBIRTH of hal
*            comes when every source reads. Metric = key with '.' -> '/'. Real: published
*            round(v/q)*q when |v - last_sent| >= db (db defaults to q); bits/ints: any change
*            (or > db). dir=out: DCMD spBv1//0.<group>.DCMD.<node>.hal writes it by name or
*            alias with halcmd's rules (sig=: a signal without a writer, `sets`; src=: an
*            unlinked IN pin, `setp`); a message is checked whole before any write; the
*            value goes out read back by the next DDATA.
*     hz <f>                read rate of the hal sources (else every publisher period)
*     spb_keepalive_s <s>   an empty DDATA of hal after s of silence
*     scope_channel <n> | scope_batch <n> | scope_dt_ns <n> | scope_type double|float |
*     scope_col <Key.chan> [scale]
*            Device scope: the FIFO of `sampler` channel n (hal_stream, as halsampler; one
*            reader per FIFO), attached once the sampler component exists, the tail queued at
*            attach dropped, then batches of scope_batch rows as one DDATA of
*            DoubleArray/FloatArray columns (packed LE) stamped with the first row's time; a
*            column constant at its last sent value is left out (report by exception).
*   hal2spb's runtime keys (url user pass node spb_group spb_birth_s format scope_subject)
*   are ignored here: the environment / INI [NATS] give them.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#include "halnats.hh"

#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <thread>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <inifile.hh>
#include <nats/nats.h>
#include "hal.h"
#include "../../components/streamer.h"   // SAMPLER_SHMEM_KEY - sampler.c's FIFO key

using linuxcnc::IniFile;

namespace halnats {

typedef SparkplugB__Payload Payload;

namespace {

Config cfg;
Owner *own = nullptr;

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

uint64_t hal_alias0 = 0, scope_alias0 = 0;      // after the owner's 1..N

std::mutex mx;
natsConnection *nc = nullptr;
natsSubscription *snap_sub = nullptr, *ncmd_sub = nullptr, *dcmd_sub = nullptr;
std::vector<natsSubscription *> own_dcmd_subs;   // DCMD of the owner devices that take commands
std::vector<size_t> own_dcmd_dev;                // closure of each: the device index (stable storage)
std::thread pub_thread, scope_thread;
std::atomic<bool> running{false}, need_birth{false};
bool enabled = false;
bool born = false;                               // under mx: births of this connection sent
int64_t bdseq = 0;
uint64_t seq = 0;
double last_pub = 0;                             // under mx: CLOCK_MONOTONIC of the last seq message
std::string s_nbirth, s_ndeath, s_ncmd, s_dcmd_hal, s_snapshot;
std::vector<std::string> s_dbirth, s_ddata;      // owner devices, then hal, scope
size_t d_hal = 0, d_scope = 0;

const char *tag() { return cfg.tag.empty() ? "halnats" : cfg.tag.c_str(); }

} // namespace

void say(bool error, const char *fmt, ...)
{
    char ln[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ln, sizeof ln, fmt, ap);
    va_end(ap);
    if (cfg.log) cfg.log(error, ln);
    else fputs(ln, stderr);
}

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

void set_value(Metric *m, uint32_t dt, Val v)
{
    if (dt == HN_DT(Double)) { m->value_case = HN_V(DOUBLE_VALUE); m->double_value = v.d; }
    else if (dt == HN_DT(Boolean)) { m->value_case = HN_V(BOOLEAN_VALUE); m->boolean_value = v.i != 0; }
    else if (dt == HN_DT(Int64) || dt == HN_DT(UInt64)) { m->value_case = HN_V(LONG_VALUE); m->long_value = (uint64_t)v.i; }
    else { m->value_case = HN_V(INT_VALUE); m->int_value = (uint32_t)(int32_t)v.i; }   // Int32 two's complement
}

Metric &Batch::add(const std::string &name, uint64_t alias, uint32_t dt, uint64_t ts)
{
    const Metric m0 = SPARKPLUG_B__PAYLOAD__METRIC__INIT;
    ms.push_back(m0);
    Metric &m = ms.back();
    if (!name.empty()) { names.push_back(name); m.name = (char *)names.back().c_str(); }
    if (alias) { m.has_alias = 1; m.alias = alias; }
    if (dt) { m.has_datatype = 1; m.datatype = dt; }
    if (ts) { m.has_timestamp = 1; m.timestamp = ts; }
    return m;
}

namespace {

void send(const std::string &subj, Batch &b, bool with_seq)
{
    std::vector<Metric *> mp(b.ms.size());
    for (size_t k = 0; k < b.ms.size(); k++) mp[k] = &b.ms[k];
    Payload pl = SPARKPLUG_B__PAYLOAD__INIT;
    pl.has_timestamp = 1; pl.timestamp = now_ms();
    pl.n_metrics = mp.size(); pl.metrics = mp.data();
    if (with_seq) { pl.has_seq = 1; pl.seq = seq; seq = (seq + 1) & 0xff; last_pub = now_mono(); }
    size_t n = sparkplug_b__payload__get_packed_size(&pl);
    std::vector<uint8_t> buf(n);
    sparkplug_b__payload__pack(&pl, buf.data());
    natsConnection_Publish(nc, subj.c_str(), buf.data(), (int)n);
}

// ---- device hal ----

uint32_t hal_dt(hal_type_t t)
{
    return t == HAL_BOOL ? HN_DT(Boolean) : t == HAL_REAL ? HN_DT(Double) : t == HAL_UINT ? HN_DT(UInt64) : HN_DT(Int64);
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
            if (!p.said) say(false, "%s: NATS hal %s %s: %s - DBIRTH of hal waits\n",
                             tag(), p.sig ? "signal" : "pin", p.src.c_str(), hal_err(rc));
            p.said = true;
            continue;
        }
        if (p.said && !p.ok) say(false, "%s: NATS hal %s now reads\n", tag(), p.src.c_str());
        p.v = v; p.ts = ts; p.ok = true;
    }
}
bool hal_all_ok()
{
    for (const HPin &p : hpins) if (!p.ok) return false;
    return true;
}
void hal_metric(Batch &b, size_t k, bool named, bool prefixed)
{
    HPin &p = hpins[k];
    Metric &m = b.add(named ? (prefixed ? "hal/" + p.metric : p.metric) : std::string(),
                      hal_alias0 + k, named ? hal_dt(p.type) : 0, p.ts);
    set_value(&m, hal_dt(p.type), quant(p, p.v));
}
void hal_birth()
{
    Batch b;
    for (size_t k = 0; k < hpins.size(); k++) {
        hal_metric(b, k, true, false);
        hpins[k].last = quant(hpins[k], hpins[k].v); hpins[k].sent = true;
    }
    send(s_dbirth[d_hal], b, true);
    hal_born = true;
}
void hal_datas()
{
    Batch b;
    for (size_t k = 0; k < hpins.size(); k++) {
        HPin &p = hpins[k];
        if (!p.ok || !moved(p, p.v)) continue;
        hal_metric(b, k, false, false);
        p.last = quant(p, p.v); p.sent = true;
    }
    // empty DDATA = keepalive: a node standing still is not silent past spb_keepalive_s
    if (!b.ms.empty() || (keepalive_s > 0 && now_mono() - last_pub >= keepalive_s))
        send(s_ddata[d_hal], b, true);
}

// ---- device scope (declared in births, data from scope_main) ----

void scope_birth()
{
    Batch b;
    Metric &m = b.add("dt_ns", scope_alias0, HN_DT(Int64), 0);
    m.value_case = HN_V(LONG_VALUE); m.long_value = (uint64_t)s_dt_ns;
    for (size_t c = 0; c < scols.size(); c++) {  // declared, empty array: values come in DDATA
        Metric &mc = b.add(scols[c].key + "/" + scols[c].ch, scope_alias0 + 1 + c,
                           s_float ? HN_DT(FloatArray) : HN_DT(DoubleArray), 0);
        mc.value_case = HN_V(BYTES_VALUE); mc.bytes_value.len = 0; mc.bytes_value.data = nullptr;
    }
    send(s_dbirth[d_scope], b, true);
}

void births()
{
    seq = 0;
    Batch nb;
    Metric &m0 = nb.add("bdSeq", 0, HN_DT(Int64), 0);
    m0.value_case = HN_V(LONG_VALUE); m0.long_value = bdseq;
    Metric &m1 = nb.add("Node Control/Rebirth", 0, HN_DT(Boolean), 0);
    m1.value_case = HN_V(BOOLEAN_VALUE); m1.boolean_value = 0;
    send(s_nbirth, nb, true);
    if (own)
        for (size_t d = 0; d < own->devices().size(); d++) {
            Batch b;
            own->birth(d, b);
            send(s_dbirth[d], b, true);
        }
    born = true;
    hal_born = false;
    if (!hpins.empty() && hal_all_ok()) hal_birth();   // else when every source reads (publisher)
    if (s_chan >= 0) scope_birth();
}

void on_snapshot(natsConnection *, natsSubscription *, natsMsg *msg, void *)
{
    const char *reply = natsMsg_GetReply(msg);
    if (reply && *reply) {
        std::string want(natsMsg_GetData(msg) ? natsMsg_GetData(msg) : "", natsMsg_GetDataLength(msg));
        std::string r(reply);
        std::lock_guard<std::mutex> g(mx);
        Batch b;
        if (own) own->snapshot(want, b);
        for (size_t k = 0; k < hpins.size(); k++)
            if (hpins[k].ok && (want.empty() || want == "hal")) hal_metric(b, k, true, true);
        send(r, b, false);                           // a reply, not part of the seq stream
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
    case HN_V(BOOLEAN_VALUE):
        if (dt && dt != HN_DT(Boolean)) return "datatype/value mismatch";
        k = K_B; i = m->boolean_value != 0; break;
    case HN_V(DOUBLE_VALUE): k = K_R; r = m->double_value; break;
    case HN_V(FLOAT_VALUE): k = K_R; r = m->float_value; break;
    case HN_V(INT_VALUE):   // Int8/16/32 two's complement, UInt8/16/32
        k = K_I; i = (dt >= HN_DT(UInt8) && dt <= HN_DT(UInt32)) ? (int64_t)m->int_value : (int64_t)(int32_t)m->int_value; break;
    case HN_V(LONG_VALUE):
        if (dt == HN_DT(UInt64)) { k = K_U; u = m->long_value; } else { k = K_I; i = (int64_t)m->long_value; }
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
    if (!pl) { say(true, "%s: NATS DCMD hal refused: not a Sparkplug B payload\n", tag()); return; }
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
    if (bad) { say(true, "%s: NATS DCMD hal refused (%zu cmds): %s: %s\n", tag(), cmds.size(), who.c_str(), bad); return; }
    for (const Cmd &c : cmds) {
        hal_query_t q;
        memset(&q, 0, sizeof q);
        q.name = c.p->src.c_str();
        int rc;
        if (c.p->sig) { q.qtype = HAL_QTYPE_SIGNAL; q.sig.type = c.p->type; q.sig.value = c.v; rc = hal_set_s(&q, nullptr, nullptr); }
        else { q.qtype = HAL_QTYPE_PIN; q.pp.type = c.p->type; q.pp.value = c.v; rc = hal_set_p(&q, nullptr, nullptr); }
        if (rc) say(true, "%s: NATS DCMD hal %s -> %s: %s\n", tag(), c.p->metric.c_str(), c.p->src.c_str(), hal_err(rc));
    }
}

// DCMD of an owner device: handed to the owner as is (it checks and queues; never under mx)
void on_own_dcmd(natsConnection *, natsSubscription *, natsMsg *msg, void *closure)
{
    size_t d = *(const size_t *)closure;
    Payload *pl = sparkplug_b__payload__unpack(nullptr, (size_t)natsMsg_GetDataLength(msg),
                                               (const uint8_t *)natsMsg_GetData(msg));
    natsMsg_Destroy(msg);
    if (!pl) { say(true, "%s: NATS DCMD %s refused: not a Sparkplug B payload\n", tag(), own->devices()[d].c_str()); return; }
    own->command(d, pl);
    sparkplug_b__payload__free_unpacked(pl, nullptr);
}

void destroy_subs()
{
    natsSubscription_Destroy(snap_sub); snap_sub = nullptr;
    natsSubscription_Destroy(ncmd_sub); ncmd_sub = nullptr;
    natsSubscription_Destroy(dcmd_sub); dcmd_sub = nullptr;
    for (natsSubscription *&s : own_dcmd_subs) { natsSubscription_Destroy(s); s = nullptr; }
    own_dcmd_subs.clear();
}

// NCMD: "Node Control/Rebirth" only (HAL commands are DCMD of device hal)
void on_ncmd(natsConnection *, natsSubscription *, natsMsg *msg, void *)
{
    Payload *pl = sparkplug_b__payload__unpack(nullptr, (size_t)natsMsg_GetDataLength(msg),
                                               (const uint8_t *)natsMsg_GetData(msg));
    natsMsg_Destroy(msg);
    if (!pl) { say(true, "%s: NATS NCMD refused: not a Sparkplug B payload\n", tag()); return; }
    for (size_t n = 0; n < pl->n_metrics; n++) {
        const Metric *m = pl->metrics[n];
        if (m->name && !strcmp(m->name, "Node Control/Rebirth")) { if (m->boolean_value) need_birth = true; }
        else say(true, "%s: NATS NCMD %s refused: the node takes Node Control/Rebirth; HAL commands are DCMD of device hal\n",
                 tag(), m->name ? m->name : "(alias)");
    }
    sparkplug_b__payload__free_unpacked(pl, nullptr);
}

void on_connected(natsConnection *, void *)
{
    say(false, "%s: NATS connected - Sparkplug births follow\n", tag());
    need_birth = true;
}
void on_disconnected(natsConnection *, void *)
{
    say(false, "%s: NATS disconnected\n", tag());
    std::lock_guard<std::mutex> g(mx);
    born = false;                                // no DDATA of a dead session; births on reconnect
}

void publisher()
{
    while (running) {
        {
            std::lock_guard<std::mutex> g(mx);
            if (own) own->pull();
            if (!hpins.empty() && now_mono() - hal_last_read >= hal_period_s) { hal_last_read = now_mono(); hal_poll(); }
            if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) {
                if (need_birth && (!own || own->ready())) { need_birth = false; births(); }
                else if (!need_birth && born) {
                    if (own)
                        for (size_t d = 0; d < own->devices().size(); d++) {
                            Batch b;
                            own->data(d, b);
                            if (!b.ms.empty()) send(s_ddata[d], b, true);
                        }
                    if (!hpins.empty()) {
                        if (!hal_born) { if (hal_all_ok()) hal_birth(); }
                        else hal_datas();
                    }
                }
            }
        }
        double p_s = cfg.period_s;
        struct timespec p = {(time_t)p_s, (long)((p_s - (time_t)p_s) * 1e9)};
        nanosleep(&p, nullptr);
    }
}

// The sampler FIFO -> DDATA of device scope (hal2spb's scope thread). One reader per FIFO
// (hal_stream): halsampler must not read the same channel.
void scope_main()
{
    hal_stream_t st;
    bool said = false;
    // the sampler may load after the node (HALFILEs after the task): wait for the component
    // before attaching (a quiet probe - hal_stream_attach logs every failure)
    // ПРЕДПОЛОЖЕНИЕ, не замер: a 1 s poll is fine for a FIFO that exists from HAL load on
    for (;;) {
        if (!running) return;
        hal_query_t q;
        memset(&q, 0, sizeof q);
        if (hal_comp_by_name("sampler", &q) == 0) {
            int r = hal_stream_attach(&st, hal_comp, SAMPLER_SHMEM_KEY + s_chan, nullptr);
            if (r >= 0) break;
            if (!said) say(true, "%s: NATS scope: hal_stream_attach(channel %d): %s - retrying\n", tag(), s_chan, strerror(-r));
            said = true;
        } else if (!said) {
            say(false, "%s: NATS scope: no sampler component yet - waiting\n", tag());
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
    say(false, "%s: NATS scope: sampler channel %d, %d columns (%d in the conf), dropped stale tail %d\n",
        tag(), s_chan, ne, (int)scols.size(), dropped_rows);
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
        Batch b;
        for (int c = 0; c < ncol; c++) {
            // report by exception: a column constant at its last sent value is not sent
            double *x = &col[(size_t)c * s_batch];
            bool same = sent_ok[c] && x[0] == sent_last[c];
            for (int i = 1; same && i < n; i++) same = x[i] == x[0];
            if (same) continue;
            sent_ok[c] = 1; sent_last[c] = x[n - 1];
            if (s_float) { float *f = (float *)x; for (int i = 0; i < n; i++) f[i] = (float)x[i]; }
            Metric &m = b.add(std::string(), scope_alias0 + 1 + c, 0, ts);
            m.value_case = HN_V(BYTES_VALUE);
            m.bytes_value.len = (size_t)n * (s_float ? 4 : 8);   // packed LE float32/float64
            m.bytes_value.data = (uint8_t *)x;
        }
        if (b.ms.empty()) continue;
        std::lock_guard<std::mutex> g(mx);
        // off the broker or before this session's DBIRTH: dropped (the birth declares, data follows)
        if (born && natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) send(s_ddata[d_scope], b, true);
    }
    hal_stream_detach(&st);
}

// ---- PINS: the hal2spb conf line format ----

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
        say(true, "%s: NATS PINS %s: %s - NATS publishing refused\n", tag(), path.c_str(), strerror(errno));
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
            say(true, "%s: NATS PINS %s:%d: %s - NATS publishing refused\n", tag(), path.c_str(), lineno, why.c_str());
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
                else if (!t.compare(0, 4, "hal=")) {}   // hal2spb's own pin name: the node makes no pins
                else bad("pin " + p.key + ": '" + t + "' - dir= src= sig= q= db=");
            }
            if (!good) break;
            if (p.src.empty()) { bad("pin " + p.key + ": no src=<pin> or sig=<signal> - the node reads HAL by name, it makes no pins"); break; }
            for (const HPin &o : hpins) if (o.metric == p.metric) { bad("pin " + p.key + ": a second row of the same metric"); break; }
            if (!good) break;
            if (p.type != HAL_REAL) p.q = 0;
            else if (p.q > 0 && !(p.db > 0)) p.db = p.q;   // deadband = one quantum
            if (p.type == HAL_REAL && !(p.q > 0))
                say(false, "%s: NATS PINS pin %s: no q= - sent unquantized (db=%g)\n", tag(), p.key.c_str(), p.db);
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
            ignored += " " + a;                  // hal2spb's runtime keys: the environment / INI [NATS] give them
        } else {
            bad("unknown directive '" + a + "'");
        }
    }
    fclose(f);
    if (!good) return false;
    if (!ignored.empty())
        say(false, "%s: NATS PINS %s: ignored (the environment / INI [NATS] give them):%s\n", tag(), path.c_str(), ignored.c_str());
    if (s_chan >= 0) {
        if (scols.empty() || s_batch <= 0 || s_dt_ns <= 0 || (int)scols.size() > HAL_STREAM_MAX_PINS) {
            say(true, "%s: NATS PINS %s: scope_channel needs scope_col (1..%d), scope_batch > 0 and "
                "scope_dt_ns > 0 (the sampler thread period) - NATS publishing refused\n",
                tag(), path.c_str(), HAL_STREAM_MAX_PINS);
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

std::string env(const char *var)
{
    const char *v = getenv(var);
    return v ? v : "";
}

} // namespace

int configure(Config &c, const char *inifile, const char *node_suffix, double default_period)
{
    c.url = env("SYNCTWIN_NATS_URL");
    c.user = env("SYNCTWIN_NATS_USER");
    c.pass = env("SYNCTWIN_NATS_PASS");
    c.group = env("SYNCTWIN_BOX_ID");
    c.node = env("NODE_ID");
    if (!c.node.empty() && node_suffix) c.node += node_suffix;
    c.period_s = default_period;
    if (inifile) {
        IniFile ini(inifile);
        if (ini) {
            const char *keys[] = {"URL", "USER", "PASS", "GROUP", "NODE"};
            std::string *dst[] = {&c.url, &c.user, &c.pass, &c.group, &c.node};
            for (int k = 0; k < 5; k++)
                if (auto v = ini.findString(keys[k], "NATS")) *dst[k] = *v;
            c.period_s = ini.findRealV("PUBLISH_PERIOD", "NATS", default_period);
            if (auto pins = ini.findString("PINS", "NATS")) {
                std::string path = *pins, ini_s(inifile);
                size_t sl = ini_s.rfind('/');
                if (!path.empty() && path[0] != '/' && sl != std::string::npos) path = ini_s.substr(0, sl + 1) + path;
                c.pins = path;
            }
        }
    }
    cfg.tag = c.tag; cfg.log = c.log;
    if (c.url.empty()) {
        say(false, "%s: no NATS URL (SYNCTWIN_NATS_URL, INI [NATS] URL) - publishing to NATS is off\n", tag());
        return 1;
    }
    if (!name_ok(c.group) || !name_ok(c.node)) {
        say(true, "%s: NATS group '%s' / node '%s' missing or not a Sparkplug id (no . / + # * > or spaces) - "
            "SYNCTWIN_BOX_ID / NODE_ID or INI [NATS] GROUP / NODE - publishing refused\n",
            tag(), c.group.c_str(), c.node.c_str());
        return -1;
    }
    return 0;
}

int start(const Config &c, int hal_comp_id, Owner *owner)
{
    cfg = c;
    own = owner;
    if (!cfg.pins.empty() && !load_pins(cfg.pins)) return -1;
    if (s_chan >= 0 && hal_comp_id < 0) {
        say(true, "%s: NATS PINS scope_channel without a HAL component - publishing refused\n", tag());
        return -1;
    }
    hal_comp = hal_comp_id;
    // publisher period: INI [NATS] PUBLISH_PERIOD / the owner's default, else the PINS file's hz
    if (!(cfg.period_s > 0)) cfg.period_s = hal_period_s;
    if (!(cfg.period_s > 0)) {
        say(true, "%s: NATS: no publisher period - INI [NATS] PUBLISH_PERIOD or PINS hz - publishing refused\n", tag());
        return -1;
    }
    hal_alias0 = (own ? own->aliases() : 0) + 1;
    scope_alias0 = hal_alias0 + hpins.size();

    const std::string base = "spBv1//0." + cfg.group + ".";
    s_nbirth = base + "NBIRTH." + cfg.node;
    s_ndeath = base + "NDEATH." + cfg.node;
    std::vector<std::string> devs;
    if (own) devs = own->devices();
    d_hal = devs.size(); devs.push_back("hal");
    d_scope = devs.size(); devs.push_back("scope");
    for (const std::string &d : devs) {
        s_dbirth.push_back(base + "DBIRTH." + cfg.node + "." + d);
        s_ddata.push_back(base + "DDATA." + cfg.node + "." + d);
    }
    s_ncmd = base + "NCMD." + cfg.node;
    s_dcmd_hal = base + "DCMD." + cfg.node + ".hal";
    s_snapshot = "lcnc." + cfg.group + "." + cfg.node + ".snapshot";
    bdseq = (int64_t)(now_ms() & 0xff);  // a fresh bdSeq per run (no persisted counter yet)

    natsOptions *opts = nullptr;
    natsStatus st = natsOptions_Create(&opts);
    if (st == NATS_OK) st = natsOptions_SetURL(opts, cfg.url.c_str());
    if (st == NATS_OK && !cfg.user.empty()) st = natsOptions_SetUserInfo(opts, cfg.user.c_str(), cfg.pass.c_str());
    if (st == NATS_OK) st = natsOptions_SetMaxReconnect(opts, -1);
    // connect in the background: the owner never waits on the network, not even at startup
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
    std::string own_cmds;
    if (own) {
        own_dcmd_dev.clear();
        for (size_t d = 0; d < own->devices().size(); d++) if (own->takes_commands(d)) own_dcmd_dev.push_back(d);
        own_dcmd_subs.assign(own_dcmd_dev.size(), nullptr);
        for (size_t k = 0; k < own_dcmd_dev.size() && st == NATS_OK; k++) {
            std::string subj = base + "DCMD." + cfg.node + "." + own->devices()[own_dcmd_dev[k]];
            st = natsConnection_Subscribe(&own_dcmd_subs[k], nc, subj.c_str(), on_own_dcmd, &own_dcmd_dev[k]);
            own_cmds += (own_cmds.empty() ? "" : ", ") + subj;
        }
    }
    natsOptions_Destroy(opts);
    if (st != NATS_OK) {
        say(true, "%s: NATS %s: %s - publishing refused\n", tag(), cfg.url.c_str(), natsStatus_GetText(st));
        destroy_subs();
        natsConnection_Destroy(nc); nc = nullptr;
        return -1;
    }
    if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) need_birth = true;
    enabled = true;
    running = true;
    pub_thread = std::thread(publisher);
    if (s_chan >= 0) scope_thread = std::thread(scope_main);
    for (const HPin &p : hpins)
        if (p.out) say(false, "%s: NATS hal out %s (%s %s) <- %s\n", tag(), p.metric.c_str(), p.sig ? "signal" : "pin",
                       p.src.c_str(), s_dcmd_hal.c_str());
    if (!own_cmds.empty()) say(false, "%s: NATS commands <- %s\n", tag(), own_cmds.c_str());
    say(false, "%s: NATS %s%s as Sparkplug node %s/%s, %zu own metrics, %zu hal, %zu scope columns, snapshot on %s\n",
        tag(), cfg.url.c_str(), cfg.user.empty() ? "" : " (user)", cfg.group.c_str(), cfg.node.c_str(),
        own ? own->aliases() : (size_t)0, hpins.size(), scols.size(), s_snapshot.c_str());
    return 0;
}

void stop()
{
    if (!enabled) return;
    enabled = false;
    running = false;
    if (pub_thread.joinable()) pub_thread.join();
    if (scope_thread.joinable()) scope_thread.join();
    {
        std::lock_guard<std::mutex> g(mx);
        Batch b;
        Metric &m = b.add("bdSeq", 0, HN_DT(Int64), 0);
        m.value_case = HN_V(LONG_VALUE); m.long_value = bdseq;
        if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) {
            send(s_ndeath, b, false);                // NDEATH carries no seq (6.4.5)
            // ПРЕДПОЛОЖЕНИЕ, не замер: 1 s for NDEATH to leave at shutdown
            natsConnection_FlushTimeout(nc, 1000);
        }
    }
    destroy_subs();
    natsConnection_Destroy(nc); nc = nullptr;
}

} // namespace halnats
