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
*            safe=<v> (dir=out only): the pin's safe state, Sparkplug B 3.0 Primary Host
*            awareness (spec 7.x STATE): while the Primary Host is not online - at start until
*            its first STATE online, on STATE online=false, or after spb_host_ttl_s without a
*            STATE (NATS has no retain/Will: the host repeats it) - every safe= row is written
*            its value once and DCMD to those rows is refused; STATE online again lets commands
*            through (the safe values stay until a command changes them).
*     spb_host_ttl_s <s>    silence of the Primary Host STATE that counts as offline (needed with safe=)
*     hz <f>                read rate of the hal sources (else every publisher period)
*     spb_keepalive_s <s>   an empty DDATA of hal after s of silence
*     scope_channel <n> | scope_batch <n> | scope_dt_ns <n> | scope_type double|float |
*     scope_col <Key.chan> [scale]
*            Device scope: the FIFO of `sampler` channel n (hal_stream, as halsampler; one
*            reader per FIFO), attached once the sampler component exists, the tail queued at
*            attach dropped, then batches of scope_batch rows as one DDATA of
*            DoubleArray/FloatArray columns (packed LE) stamped with the first row's time; a
*            column constant at its last sent value is left out (report by exception).
*     tag <key> bit|s32|u32|s64|float dir=in|out src=<pin> | sig=<signal> writer=<equipment> stand=<stand>
*         alive=<liveness key> [strobe=1] [safe=<v>]
*            A cell tag in KV bucket CELL - node to node, no Sparkplug (the pipe of the removed agent
*            nats-bridge, rule for rule). Bodies: cnc_servo/tag_envelope.py. dir=in (the node writes): HAL
*            read every publisher period, put $KV.CELL.<key> by exception {value, hal_type, writer, ts, seq,
*            q}, ts = the read moment, seq = boot time in us +1 per put; the liveness key of `stand` gets
*            {alive, writer: stand, ts, seq} every tag_ttl_s/3, all values again (BIRTH) before it after every
*            (re)connect, alive false at a clean exit. dir=out (the node reads): watch kv.CELL.<key> (the
*            stream's republish) and the writer stand's liveness key, current values by direct get
*            KV_CELL {"last_by_subj"} after every (re)connect; a value reaches HAL (halcmd's rules, as
*            pin dir=out) only from the bound `writer`, of its hal_type, newer seq, while the writer stand
*            is alive (liveness younger than tag_ttl_s) - a value got while it is dead is held and applied
*            on its return. Writer stand dead, its alive false or the NATS link lost: every dir=out end of
*            it gets its rest value once (safe=<v>, else the HAL default 0); its return re-reads the
*            current values. strobe=1: an event of its data (RemoteInterface CmdValue, touch-probe
*            latch-seq) - read first, put last, applied last. One stand per node for dir=in.
*     tag_ttl_s <s>         writer liveness TTL (tag_envelope.ALIVE_TTL_S; needed with tags)
*     tag_ok sig=<signal> | src=<pin>
*            bit, the quality of every read tag (the bridge's link-ok, the node's Interlock/Starved):
*            1 = link up, every writer stand alive, no refused read tag; 0 said with its reason.
*   hal2spb's runtime keys (url user pass node spb_group spb_birth_s format scope_subject)
*   are ignored here: the environment / INI [NATS] give them.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#include "halnats.hh"
#include "halnats_err.hh"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <set>
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
    bool has_safe;                               // dir=out safe=<v>: written while the Primary Host is away
    hal_query_value_u safe;
    Val v, last;                                 // last read / last sent (quantized)
    uint64_t ts;
    bool ok, sent, said;
};
std::vector<HPin> hpins;
bool hal_born = false;                           // under mx
double keepalive_s = 0;                          // spb_keepalive_s of the file; 0 = none
double hal_period_s = 0;                         // 1 / hz of the file; 0 = every publisher period
double hal_last_read = 0;

// Primary Host awareness (safe= rows): STATE spBv1//0.STATE.<host> {online, timestamp}
double host_ttl_s = 0;                           // spb_host_ttl_s of the file
std::atomic<bool> host_online{false};            // the last STATE said online (nats thread)
std::atomic<double> host_seen{0};                // CLOCK_MONOTONIC of that STATE
std::atomic<bool> held_safe{false};              // safe values written, DCMD to safe= rows refused
bool guard = false;                              // safe= rows and a Primary Host id
natsSubscription *state_sub = nullptr;

// device scope: the sampler FIFO
int s_chan = -1, s_batch = 0;
long s_dt_ns = 0;
bool s_float = false;
struct SCol { std::string key, ch; double scale; };
std::vector<SCol> scols;
std::vector<double> s_last;                      // under mx: the last value read of each scope column
uint64_t s_last_ts = 0;                          // under mx: ms of that row; 0 = nothing read yet
int hal_comp = -1;

uint64_t hal_alias0 = 0, scope_alias0 = 0;      // after the owner's 1..N

std::mutex mx;
natsConnection *nc = nullptr;
natsSubscription *snap_sub = nullptr, *ncmd_sub = nullptr, *dcmd_sub = nullptr;
std::vector<natsSubscription *> own_dcmd_subs;   // DCMD of the owner devices that take commands
std::vector<size_t> own_dcmd_dev;                // closure of each: the device index (stable storage)
std::thread pub_thread, scope_thread;
std::atomic<bool> running{false}, need_birth{false};
std::atomic<bool> need_reload{false};           // Rebirth / SIGHUP: PINS re-read, resubscribed, births (publisher)
bool enabled = false;
bool born = false;                               // under mx: births of this connection sent
int64_t bdseq = 0;
uint64_t seq = 0;
double last_pub = 0;                             // under mx: CLOCK_MONOTONIC of the last seq message
ErrWatch errw;                                   // async errors of the connection (halnats_err.hh)
uint64_t errs_sent = 0;                          // under mx: publisher/errors last put to the broker
std::string s_ndata;
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
int hal_get_named(bool sig, const std::string &src, hal_type_t type, Val &v)
{
    hal_query_t q;
    memset(&q, 0, sizeof q);
    q.name = src.c_str();
    hal_query_value_u u;
    int rc;
    if (sig) { q.qtype = HAL_QTYPE_SIGNAL; q.sig.type = type; rc = hal_get_s(&q, nullptr, nullptr); u = q.sig.value; }
    else { q.qtype = HAL_QTYPE_PIN; q.pp.type = type; rc = hal_get_p(&q, nullptr, nullptr); u = q.pp.value; }
    if (rc) return rc;
    switch (type) {
    case HAL_BOOL: v.i = u.b ? 1 : 0; break;
    case HAL_SINT: v.i = u.s; break;
    case HAL_UINT: v.i = (int64_t)u.u; break;
    default: v.d = u.r; break;
    }
    return 0;
}
int hal_read(const HPin &p, Val &v) { return hal_get_named(p.sig, p.src, p.type, v); }
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
    // Sparkplug B: the birth carries the current value of every metric - a one-element array of the last
    // row read (report by exception skips a column standing at it); nothing read yet - declared, empty
    std::vector<double> d(scols.size());
    std::vector<float> f(scols.size());
    for (size_t c = 0; c < scols.size(); c++) {
        bool have = s_last_ts && c < s_last.size();
        Metric &mc = b.add(scols[c].key + "/" + scols[c].ch, scope_alias0 + 1 + c,
                           s_float ? HN_DT(FloatArray) : HN_DT(DoubleArray), have ? s_last_ts : 0);
        mc.value_case = HN_V(BYTES_VALUE); mc.bytes_value.len = 0; mc.bytes_value.data = nullptr;
        if (!have) continue;
        d[c] = s_last[c]; f[c] = (float)s_last[c];
        mc.bytes_value.len = s_float ? 4 : 8;
        mc.bytes_value.data = s_float ? (uint8_t *)&f[c] : (uint8_t *)&d[c];
    }
    send(s_dbirth[d_scope], b, true);
}

// publisher health: errors of the broker connection and the last one; `le` keeps the string alive
void add_err_metrics(Batch &b, std::string &le)
{
    errs_sent = errw.total();
    le = errw.last();
    Metric &e = b.add("publisher/errors", 0, HN_DT(Int64), 0);
    e.value_case = HN_V(LONG_VALUE); e.long_value = errs_sent;
    Metric &l = b.add("publisher/last_error", 0, HN_DT(String), 0);
    l.value_case = HN_V(STRING_VALUE); l.string_value = (char *)le.c_str();
}

void births()
{
    seq = 0;
    Batch nb;
    Metric &m0 = nb.add("bdSeq", 0, HN_DT(Int64), 0);
    m0.value_case = HN_V(LONG_VALUE); m0.long_value = bdseq;
    Metric &m1 = nb.add("Node Control/Rebirth", 0, HN_DT(Boolean), 0);
    m1.value_case = HN_V(BOOLEAN_VALUE); m1.boolean_value = 0;
    std::string le;
    add_err_metrics(nb, le);
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

// write with halcmd's rules (sig=: `sets` on a signal without a writer; src=: `setp` on an unlinked IN pin)
int hal_set_named(bool sig, const std::string &src, hal_type_t type, hal_query_value_u v)
{
    hal_query_t q;
    memset(&q, 0, sizeof q);
    q.name = src.c_str();
    if (sig) { q.qtype = HAL_QTYPE_SIGNAL; q.sig.type = type; q.sig.value = v; return hal_set_s(&q, nullptr, nullptr); }
    q.qtype = HAL_QTYPE_PIN; q.pp.type = type; q.pp.value = v; return hal_set_p(&q, nullptr, nullptr);
}
void hal_write(const HPin &p, hal_query_value_u v, const char *why)
{
    int rc = hal_set_named(p.sig, p.src, p.type, v);
    if (rc) say(true, "%s: NATS %s hal %s -> %s: %s\n", tag(), why, p.metric.c_str(), p.src.c_str(), hal_err(rc));
}

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
    std::lock_guard<std::mutex> g(mx);           // the rows change on a Rebirth (reload, publisher thread)
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
    if (!bad && held_safe)
        for (const Cmd &c : cmds)
            if (c.p->has_safe) { who = c.p->metric; bad = "held safe: the Primary Host is not online (STATE)"; break; }
    if (bad) { say(true, "%s: NATS DCMD hal refused (%zu cmds): %s: %s\n", tag(), cmds.size(), who.c_str(), bad); return; }
    for (const Cmd &c : cmds) hal_write(*c.p, c.v, "DCMD");
}

// STATE of the Primary Host (JSON {"online": bool, "timestamp": ms}); anything else is not online
void on_state(natsConnection *, natsSubscription *, natsMsg *msg, void *)
{
    std::string d(natsMsg_GetData(msg) ? natsMsg_GetData(msg) : "", natsMsg_GetDataLength(msg));
    natsMsg_Destroy(msg);
    std::string t;
    for (char c : d) if (c != ' ' && c != '\t' && c != '\r' && c != '\n') t += c;
    host_online = t.find("\"online\":true") != std::string::npos;
    host_seen = now_mono();
}

// the guard, in the publisher loop: host away -> every safe= row its value, once per absence
void host_guard()
{
    if (!guard) return;
    bool alive = host_online && now_mono() - host_seen < host_ttl_s;
    if (!alive && !held_safe) {
        held_safe = true;
        size_t n = 0;
        for (const HPin &p : hpins) if (p.has_safe) { hal_write(p, p.safe, "safe"); n++; }
        say(true, "%s: NATS Primary Host %s not online (%s) - %zu outputs written safe, their commands refused\n",
            tag(), cfg.host.c_str(), host_seen == 0 ? "no STATE yet" : host_online ? "STATE silent" : "STATE offline", n);
    } else if (alive && held_safe) {
        held_safe = false;
        say(false, "%s: NATS Primary Host %s online - commands to the safe outputs taken again\n", tag(), cfg.host.c_str());
    }
}

// ---- cell tags: KV bucket CELL (the box NATS; a line maps it to the line domain) ----
//
// The pipe of the removed nats-bridge (agent drivers/nats_bridge.py, tag keep/vm-agent-wave2-2026-10-18), rule
// for rule: put $KV.CELL.<key> (core PUB, stored by stream KV_CELL), current value by direct get
// $JS.API.DIRECT.GET.KV_CELL {"last_by_subj"}, watch of the stream's republish kv.CELL.<key>. Bodies - the one
// home cnc_servo/tag_envelope.py: tag envelope {value, hal_type, writer, ts, seq, q} and liveness
// {alive, writer, ts, seq}. Every tag state lives in the publisher thread; the watch callback only queues.

enum TagType { T_BIT, T_S32, T_U32, T_S64, T_FLOAT };
const char *const tag_type_names[] = {"bit", "s32", "u32", "s64", "float"};
const uint64_t U32_MAX_V = 0xffffffffull;

struct TagEnd {
    std::string key;                             // KV key without $KV.CELL.
    TagType tt;
    hal_type_t type;                             // HAL type the query API reads/writes
    bool out;                                    // dir=out: KV -> HAL (a reader end); dir=in: HAL -> KV (writer)
    bool sig;
    std::string src;
    std::string writer;                          // equipment id of the writing end (envelope `writer`)
    std::string stand;                           // stand of the writer (liveness `writer`)
    bool strobe;                                 // event of its data (RemoteInterface CmdValue, latch-seq)
    hal_query_value_u rest;                      // dir=out: value while its writer is not alive (safe=, else 0)
    // writer end
    bool put_any = false, said = false;
    Val last_put{};
    // reader end
    bool has_seq = false;
    int64_t last_seq = 0;
    bool held = false;
    std::string held_data;
};
std::vector<TagEnd> tends;                       // readers and writers each data before strobes (stable)
std::map<std::string, std::string> tag_alive;    // writer stand -> its liveness key
std::map<std::string, std::string> tag_alive_of; // liveness key -> stand (reader side)
std::string tag_own_stand;                       // the stand of this node's dir=in tags
double tag_ttl_s = 0;                            // tag_ttl_s of the file (tag_envelope.ALIVE_TTL_S)
bool tag_ok_sig = false;
std::string tag_ok_src;                          // tag_ok sig=|src=: bit, the quality of every read tag
int tag_ok_last = -1;
int tag_ok_err = 0;                              // the last refusal of the tag_ok write, said once
std::vector<natsSubscription *> tag_subs;
std::mutex tq_mx;                                // the watch queue (nats thread -> publisher)
std::deque<std::pair<std::string, std::string>> tq;
std::atomic<bool> tag_start{false}, tag_lost{false};
bool tlink = false;                              // connected and the snapshot taken (publisher thread)
double tag_retry = 0;
std::map<std::string, double> alive_at;          // stand -> CLOCK_MONOTONIC of its last alive (now - age)
std::map<std::string, double> rested;            // stand -> since when its reader ends rest
std::set<std::string> revived;
std::map<std::string, std::string> rejected;     // key -> reason
std::string tag_reason;
double alive_sent = -1e18;
bool alive_born = false;                         // the first alive of this session sent
int64_t tag_seq = 0;                             // tag_envelope.Sequence: boot time in us, +1 per put

bool tag_has_readers() { for (const TagEnd &t : tends) if (t.out) return true; return false; }
int64_t tag_next_seq() { return ++tag_seq; }
double now_unix()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

// -- flat JSON object (the two bodies are flat; a nested value is skipped, as Python reads it and ignores it)
struct JVal { enum K { STR, NUM, BOOL, NUL, OTHER } k = OTHER; std::string s; double d = 0; bool b = false; bool integral = false; };

void j_ws(const std::string &t, size_t &i) { while (i < t.size() && (t[i] == ' ' || t[i] == '\t' || t[i] == '\n' || t[i] == '\r')) i++; }
bool j_str(const std::string &t, size_t &i, std::string &out)
{
    if (i >= t.size() || t[i] != '"') return false;
    for (i++; i < t.size(); i++) {
        char c = t[i];
        if (c == '"') { i++; return true; }
        if (c != '\\') { out += c; continue; }
        if (++i >= t.size()) return false;
        switch (t[i]) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'u': {
            if (i + 4 >= t.size()) return false;
            unsigned cp = (unsigned)strtoul(t.substr(i + 1, 4).c_str(), nullptr, 16);
            i += 4;
            if (cp < 0x80) out += (char)cp;
            else if (cp < 0x800) { out += (char)(0xc0 | (cp >> 6)); out += (char)(0x80 | (cp & 0x3f)); }
            else { out += (char)(0xe0 | (cp >> 12)); out += (char)(0x80 | ((cp >> 6) & 0x3f)); out += (char)(0x80 | (cp & 0x3f)); }
            break;
        }
        default: return false;
        }
    }
    return false;
}
bool j_skip(const std::string &t, size_t &i)          // a nested object/array
{
    int depth = 0;
    for (; i < t.size(); i++) {
        char c = t[i];
        if (c == '"') { std::string dummy; if (!j_str(t, i, dummy)) return false; i--; continue; }
        if (c == '{' || c == '[') depth++;
        else if (c == '}' || c == ']') { if (--depth == 0) { i++; return true; } }
    }
    return false;
}
bool json_flat(const std::string &t, std::map<std::string, JVal> &o)
{
    size_t i = 0;
    j_ws(t, i);
    if (i >= t.size() || t[i++] != '{') return false;
    j_ws(t, i);
    if (i < t.size() && t[i] == '}') return true;
    for (;;) {
        std::string k;
        j_ws(t, i);
        if (!j_str(t, i, k)) return false;
        j_ws(t, i);
        if (i >= t.size() || t[i++] != ':') return false;
        j_ws(t, i);
        if (i >= t.size()) return false;
        JVal v;
        char c = t[i];
        if (c == '"') { v.k = JVal::STR; if (!j_str(t, i, v.s)) return false; }
        else if (c == '{' || c == '[') { v.k = JVal::OTHER; if (!j_skip(t, i)) return false; }
        else if (!t.compare(i, 4, "true")) { v.k = JVal::BOOL; v.b = true; i += 4; }
        else if (!t.compare(i, 5, "false")) { v.k = JVal::BOOL; i += 5; }
        else if (!t.compare(i, 4, "null")) { v.k = JVal::NUL; i += 4; }
        else if (!t.compare(i, 3, "NaN")) { v.k = JVal::NUM; v.d = NAN; i += 3; }               // Python json.dumps
        else if (!t.compare(i, 8, "Infinity")) { v.k = JVal::NUM; v.d = INFINITY; i += 8; }
        else if (!t.compare(i, 9, "-Infinity")) { v.k = JVal::NUM; v.d = -INFINITY; i += 9; }
        else {
            size_t b = i;
            while (i < t.size() && (isdigit((unsigned char)t[i]) || t[i] == '-' || t[i] == '+' || t[i] == '.' || t[i] == 'e' || t[i] == 'E')) i++;
            std::string n = t.substr(b, i - b);
            if (n.empty()) return false;
            v.k = JVal::NUM; v.s = n;
            v.integral = n.find_first_of(".eE") == std::string::npos;   // Python: type(value) is int
            char *e = nullptr;
            v.d = strtod(n.c_str(), &e);
            if (!e || *e) return false;
        }
        o[k] = v;
        j_ws(t, i);
        if (i >= t.size()) return false;
        if (t[i] == ',') { i++; continue; }
        if (t[i] == '}') return true;
        return false;
    }
}
bool j_int(const JVal &v, int64_t &out)               // an exact JSON integer (or its decimal string)
{
    const std::string &s = v.s;
    if (!((v.k == JVal::NUM && v.integral) || v.k == JVal::STR) || s.empty()) return false;
    char *e = nullptr;
    errno = 0;
    long long n = strtoll(s.c_str(), &e, 10);
    if (!e || *e || errno) return false;
    out = n;
    return true;
}
std::string j_esc(const std::string &s)
{
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if ((unsigned char)c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += c;
    }
    return o;
}
std::string j_num(double d)                            // Python json.dumps of a float
{
    if (std::isnan(d)) return "NaN";
    if (std::isinf(d)) return d > 0 ? "Infinity" : "-Infinity";
    char b[40];
    snprintf(b, sizeof b, "%.17g", d);
    std::string s(b);
    if (s.find_first_of(".eE") == std::string::npos) s += ".0";
    return s;
}

// -- the envelope (tag_envelope.pack / unpack / value_of / pack_alive / is_alive)
// nullptr = packed; else the refusal (u32 out of range - a wrap would hand the reader another number)
const char *tag_pack(const TagEnd &t, Val v, double ts, int64_t seq, std::string &out)
{
    std::string val;
    char b[32];
    switch (t.tt) {
    case T_BIT: val = v.i ? "true" : "false"; break;
    case T_S32: snprintf(b, sizeof b, "%lld", (long long)v.i); val = b; break;
    case T_U32:
        if ((uint64_t)v.i > U32_MAX_V) return "value out of u32";
        snprintf(b, sizeof b, "%llu", (unsigned long long)(uint64_t)v.i); val = b; break;
    case T_S64: snprintf(b, sizeof b, "\"%lld\"", (long long)v.i); val = b; break;   // proto3 JSON int64 -> string
    default: val = j_num(v.d); break;
    }
    char tail[96];
    snprintf(tail, sizeof tail, ", \"ts\": %.6f, \"seq\": %lld, \"q\": \"good\"}", ts, (long long)seq);
    out = "{\"value\": " + val + ", \"hal_type\": \"" + tag_type_names[t.tt] + "\", \"writer\": \"" + j_esc(t.writer) + "\"" + tail;
    return nullptr;
}
std::string tag_pack_alive(const std::string &stand, bool alive, double ts, int64_t seq)
{
    char tail[80];
    snprintf(tail, sizeof tail, ", \"ts\": %.6f, \"seq\": %lld}", ts, (long long)seq);
    return std::string("{\"alive\": ") + (alive ? "true" : "false") + ", \"writer\": \"" + j_esc(stand) + "\"" + tail;
}
bool tag_is_alive(const std::string &data, const std::string &stand)
{
    std::map<std::string, JVal> o;
    if (!json_flat(data, o)) return false;
    auto a = o.find("alive"), w = o.find("writer");
    return a != o.end() && a->second.k == JVal::BOOL && a->second.b
        && w != o.end() && w->second.k == JVal::STR && w->second.s == stand;
}
// unpack + the writer check + value_of: nullptr = `v` and `seq` hold it; else the refusal
const char *tag_unpack(const TagEnd &t, const std::string &data, hal_query_value_u &v, int64_t &seq, std::string &why)
{
    std::map<std::string, JVal> o;
    if (!json_flat(data, o)) return "not an envelope in KV (not a JSON object)";
    for (const char *k : {"value", "hal_type", "writer", "ts", "seq"})
        if (!o.count(k)) { why = std::string("not an envelope in KV (no ") + k + ")"; return why.c_str(); }
    const JVal &js = o["seq"], &jt = o["ts"];
    if (!j_int(js, seq)) {
        if (js.k == JVal::NUM && std::isfinite(js.d)) seq = (int64_t)js.d;   // Python int(float) truncates
        else return "not an envelope in KV (seq is not a number)";
    }
    if (!(jt.k == JVal::NUM || (jt.k == JVal::STR && !jt.s.empty() && strtod(jt.s.c_str(), nullptr) == strtod(jt.s.c_str(), nullptr))))
        return "not an envelope in KV (ts is not a number)";
    const JVal &w = o["writer"];
    if (w.k != JVal::STR || w.s != t.writer) {
        why = "writer '" + (w.k == JVal::STR ? w.s : std::string("?")) + "' is not the bound writer '" + t.writer + "' - put refused";
        return why.c_str();
    }
    const JVal &ht = o["hal_type"], &val = o["value"];
    if (ht.k != JVal::STR || ht.s != tag_type_names[t.tt]) {
        why = "writer sends '" + (ht.k == JVal::STR ? ht.s : std::string("?")) + "', the reader is " + tag_type_names[t.tt];
        return why.c_str();
    }
    int64_t n = 0;
    switch (t.tt) {
    case T_BIT:
        if (val.k != JVal::BOOL) return "value is not bit";
        v.b = val.b; return nullptr;
    case T_S32:
        if (val.k != JVal::NUM || !j_int(val, n)) return "value is not s32";
        v.s = n; return nullptr;
    case T_U32:
        if (val.k != JVal::NUM || !j_int(val, n) || n < 0 || (uint64_t)n > U32_MAX_V) return "value is not u32 (0..2^32-1)";
        v.u = (uint64_t)n; return nullptr;
    case T_S64:
        if (!j_int(val, n)) return "value is not s64";
        v.s = n; return nullptr;
    default:
        if (val.k != JVal::NUM) return "value is not float";
        v.r = val.d; return nullptr;
    }
}

// -- KV on the node's connection
void kv_put(const std::string &key, const std::string &body)
{
    natsConnection_Publish(nc, ("$KV.CELL." + key).c_str(), body.data(), (int)body.size());
}
double rfc3339(const char *s)                          // Nats-Time-Stamp -> unix seconds; 0 = unknown
{
    struct tm tm;
    memset(&tm, 0, sizeof tm);
    int n = 0;
    if (!s || sscanf(s, "%d-%d-%dT%d:%d:%d%n", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec, &n) < 6)
        return 0;
    tm.tm_year -= 1900; tm.tm_mon -= 1;
    double frac = 0;
    if (s[n] == '.') frac = strtod(s + n, nullptr);
    return (double)timegm(&tm) + frac;
}
// 1 = got, 0 = no value in the bucket, -1 = the request failed (said)
int kv_get(const std::string &key, std::string &data, double &stored)
{
    std::string req = "{\"last_by_subj\":\"$KV.CELL." + key + "\"}";
    natsMsg *r = nullptr;
    // ПРЕДПОЛОЖЕНИЕ, не замер: 1 s per direct get - the bridge's nats_mini.direct_last default
    natsStatus st = natsConnection_Request(&r, nc, "$JS.API.DIRECT.GET.KV_CELL", req.data(), (int)req.size(), 1000);
    if (st != NATS_OK) {
        say(true, "%s: NATS tags: direct get KV_CELL %s: %s\n", tag(), key.c_str(), natsStatus_GetText(st));
        return -1;
    }
    const char *status = nullptr;
    if (natsMsgHeader_Get(r, "Status", &status) == NATS_OK && status && *status) {
        int code = atoi(status);
        natsMsg_Destroy(r);
        if (code == 404) return 0;
        say(true, "%s: NATS tags: direct get KV_CELL %s: status %d\n", tag(), key.c_str(), code);
        return -1;
    }
    data.assign(natsMsg_GetData(r) ? natsMsg_GetData(r) : "", natsMsg_GetDataLength(r));
    const char *ts = nullptr;
    stored = natsMsgHeader_Get(r, "Nats-Time-Stamp", &ts) == NATS_OK ? rfc3339(ts) : 0;
    natsMsg_Destroy(r);
    return 1;
}
void on_tag_msg(natsConnection *, natsSubscription *, natsMsg *msg, void *)
{
    const char *subj = natsMsg_GetSubject(msg);
    std::string key = subj && !strncmp(subj, "kv.CELL.", 8) ? subj + 8 : "";
    std::string data(natsMsg_GetData(msg) ? natsMsg_GetData(msg) : "", natsMsg_GetDataLength(msg));
    natsMsg_Destroy(msg);
    if (key.empty()) return;
    std::lock_guard<std::mutex> g(tq_mx);
    tq.emplace_back(key, data);
}

// -- reader: KV -> HAL
bool stand_alive(const std::string &stand)
{
    auto a = alive_at.find(stand);
    return a != alive_at.end() && now_mono() - a->second <= tag_ttl_s;
}
void tag_set(TagEnd &t, hal_query_value_u v, const char *why)
{
    int rc = hal_set_named(t.sig, t.src, t.type, v);
    if (rc) say(true, "%s: NATS tag %s %s -> %s: %s\n", tag(), why, t.key.c_str(), t.src.c_str(), hal_err(rc));
}
void tag_reject(const TagEnd &t, const std::string &why)
{
    auto r = rejected.find(t.key);
    if (r != rejected.end() && r->second == why) return;
    rejected[t.key] = why;
    say(true, "%s: NATS tag %s: %s\n", tag(), t.key.c_str(), why.c_str());
}
void tag_apply_one(TagEnd &t, const std::string &data)
{
    hal_query_value_u v;
    memset(&v, 0, sizeof v);
    int64_t seq = 0;
    std::string why;
    const char *bad = tag_unpack(t, data, v, seq, why);
    if (bad) {
        // the bridge checks seq before the type: an older value of a wrong type is refused here, not skipped
        tag_reject(t, bad);
        return;
    }
    if (t.has_seq && seq <= t.last_seq) return;  // not newer (P-5)
    tag_set(t, v, "KV");
    t.has_seq = true; t.last_seq = seq;
    rejected.erase(t.key);
}
bool tag_newer(const std::string &data, const TagEnd &t)
{
    if (!t.held) return true;
    std::map<std::string, JVal> a, b;
    int64_t sa = 0, sb = 0;
    if (!json_flat(data, a) || !json_flat(t.held_data, b) || !j_int(a["seq"], sa) || !j_int(b["seq"], sb)) return true;
    return sa > sb;
}
void tag_apply(const std::string &key, const std::string &data, double age)
{
    auto st = tag_alive_of.find(key);
    if (st != tag_alive_of.end()) {
        const std::string &stand = st->second;
        if (tag_is_alive(data, stand)) {
            alive_at[stand] = now_mono() - age;
            if (stand_alive(stand)) {
                auto r = rested.find(stand);
                if (r != rested.end()) {            // first alive after a death: GOOD again (Sparkplug B)
                    say(false, "%s: NATS tags: writer stand %s alive again (key %s) after %.1f s at rest\n",
                        tag(), stand.c_str(), key.c_str(), now_mono() - r->second);
                    rested.erase(r);
                    revived.insert(stand);
                }
                for (TagEnd &t : tends)             // data before strobe (load order), not arrival order
                    if (t.out && t.stand == stand && t.held) { t.held = false; tag_apply_one(t, t.held_data); }
            }
        } else {
            alive_at.erase(stand);                  // NDEATH: clean exit of the writer
        }
        return;
    }
    for (TagEnd &t : tends) {
        if (!t.out || t.key != key) continue;
        if (stand_alive(t.stand)) tag_apply_one(t, data);
        else if (tag_newer(data, t)) { t.held = true; t.held_data = data; }   // writer dead: not a fact of this run yet
    }
}
// every loop: reader ends of a writer stand that is not alive now rest, once per death
void tag_expire()
{
    for (TagEnd &t : tends) {
        if (!t.out || rested.count(t.stand) || stand_alive(t.stand)) continue;
        const std::string stand = t.stand;
        for (TagEnd &x : tends)
            if (x.out && x.stand == stand) { tag_set(x, x.rest, "rest"); x.has_seq = false; }
        auto a = alive_at.find(stand);
        if (a == alive_at.end())
            say(true, "%s: NATS tags: writer stand %s not alive (key %s, no alive) - its reader ends at rest\n",
                tag(), stand.c_str(), tag_alive[stand].c_str());
        else
            say(true, "%s: NATS tags: writer stand %s dead (key %s, last alive %.1f s ago) - its reader ends at rest\n",
                tag(), stand.c_str(), tag_alive[stand].c_str(), now_mono() - a->second);
        rested[stand] = now_mono();
    }
}
// direct gets of `keys`: liveness keys, then strobes, then data (data got later is at least as new as the
// strobe got); handed back in the asked order. false = a request failed (the link)
bool tag_snapshot(const std::vector<std::string> &keys, std::vector<std::pair<std::string, std::pair<std::string, double>>> &got)
{
    std::map<std::string, int> rank;
    for (const TagEnd &t : tends) if (t.out) rank[t.key] = t.strobe ? 1 : 2;
    std::vector<std::string> order;
    for (const std::string &k : keys) if (std::find(order.begin(), order.end(), k) == order.end()) order.push_back(k);
    std::vector<std::string> ask = order;
    std::stable_sort(ask.begin(), ask.end(), [&](const std::string &a, const std::string &b) { return rank[a] < rank[b]; });
    std::map<std::string, std::pair<std::string, double>> have;
    for (const std::string &k : ask) {
        std::string d;
        double stored = 0;
        int r = kv_get(k, d, stored);
        if (r < 0) return false;
        if (r > 0) have[k] = {d, stored};
    }
    for (const std::string &k : order) if (have.count(k)) got.push_back({k, have[k]});
    return true;
}
void tag_revive()
{
    std::set<std::string> stands;
    stands.swap(revived);
    std::vector<std::string> keys;
    for (const TagEnd &t : tends) if (t.out && stands.count(t.stand)) keys.push_back(t.key);
    std::vector<std::pair<std::string, std::pair<std::string, double>>> got;
    if (!tag_snapshot(keys, got)) { revived.insert(stands.begin(), stands.end()); return; }
    for (auto &g : got)
        for (TagEnd &t : tends)
            if (t.out && t.key == g.first && stand_alive(t.stand)) tag_apply_one(t, g.second.first);
}

// -- writer: HAL -> KV. Strobes are READ first and PUT last: data put before a strobe is never older than it.
void tag_push()
{
    struct R { Val v; double ts; bool ok; };
    std::vector<R> rd(tends.size());
    for (int pass = 0; pass < 2; pass++)
        for (size_t k = 0; k < tends.size(); k++) {
            TagEnd &t = tends[k];
            if (t.out || t.strobe != (pass == 0)) continue;
            Val v{};
            int rc = hal_get_named(t.sig, t.src, t.type, v);
            rd[k] = R{v, now_unix(), rc == 0};
            if (rc && !t.said) say(false, "%s: NATS tag %s: %s %s: %s - not put yet\n", tag(), t.key.c_str(),
                                   t.sig ? "signal" : "pin", t.src.c_str(), hal_err(rc));
            t.said = rc != 0;
        }
    for (size_t k = 0; k < tends.size(); k++) {   // load order: data first
        TagEnd &t = tends[k];
        if (t.out || !rd[k].ok) continue;
        Val v = rd[k].v;
        bool same = t.put_any && (t.tt == T_FLOAT ? (v.d == t.last_put.d || (std::isnan(v.d) && std::isnan(t.last_put.d)))
                                                  : v.i == t.last_put.i);
        if (same) continue;
        std::string body;
        if (const char *bad = tag_pack(t, v, rd[k].ts, tag_next_seq(), body)) {
            if (!t.said) say(true, "%s: NATS tag %s: %s - not put\n", tag(), t.key.c_str(), bad);
            t.said = true;
            continue;
        }
        kv_put(t.key, body);
        t.last_put = v; t.put_any = true;
    }
}
// every dir=in end put in this session (its source read): only then the stand says alive - a reader takes
// the values in KV as of the writer's current life once it is alive, so an alive before them would hand it
// the values of the PREVIOUS run (the bridge owned its pins and always read them; halnats waits for HAL)
bool tag_all_put()
{
    for (const TagEnd &t : tends) if (!t.out && !t.put_any) return false;
    return true;
}
void tag_keepalive(bool alive)
{
    if (tag_own_stand.empty() || (alive && !tag_all_put())) return;
    kv_put(tag_alive[tag_own_stand], tag_pack_alive(tag_own_stand, alive, now_unix(), tag_next_seq()));
    alive_sent = now_mono();
}
// quality of every read tag (bridge link-ok): link up, every writer stand alive, no rejected read tag
bool tag_quality()
{
    if (!tag_has_readers()) { tag_reason.clear(); return tlink || tends.empty(); }
    if (!tlink) { tag_reason = "NATS link down"; return false; }
    std::string dead;
    for (const auto &a : tag_alive_of) if (!stand_alive(a.second)) dead += (dead.empty() ? "" : ", ") + a.second;
    if (!dead.empty()) tag_reason = "writer stand(s) dead: " + dead;
    else if (!rejected.empty()) tag_reason = rejected.begin()->first + ": " + rejected.begin()->second;
    else tag_reason.clear();
    return tag_reason.empty();
}
// (re)connected: subscribed already (libnats resubscribes) - read current, then BIRTH the writer ends:
// values first, then the alive key (a reader holding a stale value takes them first)
bool tag_birth()
{
    std::vector<std::string> keys;
    for (const auto &a : tag_alive_of) keys.push_back(a.first);
    for (const TagEnd &t : tends) if (t.out) keys.push_back(t.key);
    std::vector<std::pair<std::string, std::pair<std::string, double>>> got;
    if (!tag_snapshot(keys, got)) return false;
    double now = now_unix();
    for (auto &g : got) {
        double stored = g.second.second;
        tag_apply(g.first, g.second.first, stored > 0 ? std::max(0.0, now - stored) : tag_ttl_s + 1);
    }
    for (TagEnd &t : tends) t.put_any = false;
    tag_push();
    alive_sent = -1e18;
    tag_keepalive(true);
    alive_born = tag_all_put();
    return true;
}
void tags_step()
{
    if (tends.empty()) return;
    bool conn = natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED;
    if (tag_lost.exchange(false) || (!conn && tlink)) {
        // the link itself lost: no alive arrives - every writer is not alive now (rest in this loop)
        tlink = false;
        alive_at.clear();
    }
    if (conn && !tlink && tag_start && now_mono() >= tag_retry) {
        if (tag_birth()) { tag_start = false; tlink = true; say(false, "%s: NATS tags: snapshot taken, %zu ends\n", tag(), tends.size()); }
        else tag_retry = now_mono() + 1.0;          // the bridge's RETRY_S
    }
    std::deque<std::pair<std::string, std::string>> q;
    { std::lock_guard<std::mutex> g(tq_mx); q.swap(tq); }
    for (auto &m : q) tag_apply(m.first, m.second, 0);
    if (conn && tlink && !revived.empty()) tag_revive();
    tag_expire();
    if (conn && tlink) {
        tag_push();
        // the first alive of a session right after the last value, then every TTL/3
        if (!tag_own_stand.empty() && (now_mono() - alive_sent >= tag_ttl_s / 3 || !alive_born)) {
            tag_keepalive(true);
            alive_born = alive_sent >= 0 && tag_all_put();
        }
    }
    if (!tag_ok_src.empty()) {
        std::string before = tag_reason;
        int ok = tag_quality() ? 1 : 0;
        if (ok != tag_ok_last) {
            hal_query_value_u v;
            memset(&v, 0, sizeof v);
            v.b = ok;
            int rc = hal_set_named(tag_ok_sig, tag_ok_src, HAL_BOOL, v);
            if (rc) { if (rc != tag_ok_err) say(true, "%s: NATS tag_ok -> %s: %s\n", tag(), tag_ok_src.c_str(), hal_err(rc)); tag_ok_err = rc; }
            else {
                tag_ok_err = 0;
                if (ok) say(false, "%s: NATS tags: tag_ok 1\n", tag());
                else say(true, "%s: NATS tags: tag_ok 0 - %s\n", tag(), tag_reason.c_str());
                tag_ok_last = ok;
            }
        } else if (!ok && tag_reason != before) say(true, "%s: NATS tags: tag_ok 0 - %s\n", tag(), tag_reason.c_str());
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
    natsSubscription_Destroy(state_sub); state_sub = nullptr;
    for (natsSubscription *&s : tag_subs) { natsSubscription_Destroy(s); s = nullptr; }
    tag_subs.clear();
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
        if (m->name && !strcmp(m->name, "Node Control/Rebirth")) { if (m->boolean_value) need_reload = true; }
        else say(true, "%s: NATS NCMD %s refused: the node takes Node Control/Rebirth; HAL commands are DCMD of device hal\n",
                 tag(), m->name ? m->name : "(alias)");
    }
    sparkplug_b__payload__free_unpacked(pl, nullptr);
}

void on_connected(natsConnection *, void *)
{
    say(false, "%s: NATS connected - Sparkplug births follow\n", tag());
    need_birth = true;
    tag_start = true;                            // tags: snapshot + BIRTH of the writer ends
}
void on_disconnected(natsConnection *, void *)
{
    say(false, "%s: NATS disconnected\n", tag());
    errw.note("disconnect", "связь с брокером NATS потеряна");
    tag_lost = true;
    std::lock_guard<std::mutex> g(mx);
    born = false;                                // no DDATA of a dead session; births on reconnect
}

void reload();

void publisher()
{
    while (running) {
        {
            std::lock_guard<std::mutex> g(mx);
            if (need_reload.exchange(false)) reload();
            if (own) own->pull();
            host_guard();
            tags_step();
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
                    if (errw.total() != errs_sent) {     // NDATA by exception: only when the count moved
                        Batch b;
                        std::string le;
                        add_err_metrics(b, le);
                        send(s_ndata, b, true);
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
    { std::lock_guard<std::mutex> g(mx); s_last.assign(ncol > 0 ? ncol : 0, 0.0); s_last_ts = 0; }
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
        std::lock_guard<std::mutex> g(mx);       // scope_alias0 moves with the hal rows (Rebirth reload)
        s_last_ts = (uint64_t)llround(t_last * 1000.0);
        Batch b;
        for (int c = 0; c < ncol; c++) {
            // report by exception: a column constant at its last sent value is not sent (the birth carries it)
            double *x = &col[(size_t)c * s_batch];
            s_last[c] = x[n - 1];
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

// safe=<v> of a row -> the value of its HAL type; nullptr = fine
const char *safe_of(hal_type_t type, const std::string &s, hal_query_value_u &out)
{
    double v = 0;
    if (!num(s, v)) return "a number";
    if (type == HAL_BOOL) { if (v != 0 && v != 1) return "0|1"; out.b = v != 0; }
    else if (type == HAL_REAL) out.r = v;
    else if (v != std::floor(v) || (type == HAL_UINT && v < 0)) return "not its integer type";
    else if (type == HAL_UINT) out.u = (uint64_t)v;
    else out.s = (int64_t)v;
    return nullptr;
}
// a KV key: NATS tokens of the KV alphabet, joined by '.'
bool name_ok_kv(const std::string &k)
{
    if (k.empty() || k.front() == '.' || k.back() == '.' || k.find("..") != std::string::npos) return false;
    for (char c : k)
        if (!(isalnum((unsigned char)c) || c == '-' || c == '_' || c == '/' || c == '=' || c == '.')) return false;
    return true;
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
            std::string safe_s;
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
                else if (!t.compare(0, 5, "safe=")) safe_s = t.substr(5);
                else if (!t.compare(0, 4, "hal=")) {}   // hal2spb's own pin name: the node makes no pins
                else bad("pin " + p.key + ": '" + t + "' - dir= src= sig= q= db= safe=");
            }
            if (!good) break;
            if (p.src.empty()) { bad("pin " + p.key + ": no src=<pin> or sig=<signal> - the node reads HAL by name, it makes no pins"); break; }
            for (const HPin &o : hpins) if (o.metric == p.metric) { bad("pin " + p.key + ": a second row of the same metric"); break; }
            if (!good) break;
            if (!safe_s.empty()) {
                if (!p.out) { bad("pin " + p.key + ": safe= on dir=in - only a command output has a safe state"); break; }
                if (const char *e = safe_of(p.type, safe_s, p.safe)) { bad("pin " + p.key + ": safe=" + safe_s + " - " + e); break; }
                p.has_safe = true;
            }
            if (p.type != HAL_REAL) p.q = 0;
            else if (p.q > 0 && !(p.db > 0)) p.db = p.q;   // deadband = one quantum
            if (p.type == HAL_REAL && !(p.q > 0))
                say(false, "%s: NATS PINS pin %s: no q= - sent unquantized (db=%g)\n", tag(), p.key.c_str(), p.db);
            hpins.push_back(p);
        } else if (a == "tag") {
            if (w.size() < 3) { bad("tag <key> bit|s32|u32|s64|float dir=in|out src=|sig= writer= stand= alive= [strobe=1] [safe=]"); break; }
            TagEnd t{};
            t.key = w[1];
            if (!name_ok_kv(t.key)) { bad("tag " + t.key + ": not a KV key (tokens of [-_A-Za-z0-9/=] by '.')"); break; }
            int tt = -1;
            for (int k = 0; k < 5; k++) if (w[2] == tag_type_names[k]) tt = k;
            if (tt < 0) { bad("tag " + t.key + ": type '" + w[2] + "' - bit|s32|u32|s64|float (tag_envelope.HAL_TYPES)"); break; }
            t.tt = (TagType)tt;
            t.type = tt == T_BIT ? HAL_BOOL : tt == T_FLOAT ? HAL_REAL : tt == T_U32 ? HAL_UINT : HAL_SINT;
            std::string dir, safe_s, alive;
            for (size_t i = 3; i < w.size() && good; i++) {
                const std::string &x = w[i];
                if (x == "dir=in" || x == "dir=out") dir = x.substr(4);
                else if (!x.compare(0, 4, "src=")) { t.src = x.substr(4); t.sig = false; }
                else if (!x.compare(0, 4, "sig=")) { t.src = x.substr(4); t.sig = true; }
                else if (!x.compare(0, 7, "writer=")) t.writer = x.substr(7);
                else if (!x.compare(0, 6, "stand=")) t.stand = x.substr(6);
                else if (!x.compare(0, 6, "alive=")) alive = x.substr(6);
                else if (x == "strobe=1" || x == "strobe=0") t.strobe = x == "strobe=1";
                else if (!x.compare(0, 5, "safe=")) safe_s = x.substr(5);
                else bad("tag " + t.key + ": '" + x + "' - dir= src= sig= writer= stand= alive= strobe= safe=");
            }
            if (!good) break;
            t.out = dir == "out";
            if (dir.empty()) { bad("tag " + t.key + ": no dir=in|out"); break; }
            if (t.src.empty()) { bad("tag " + t.key + ": no src=<pin> or sig=<signal> - the node reads/writes HAL by name"); break; }
            if (t.writer.empty() || t.stand.empty() || alive.empty()) {
                bad("tag " + t.key + ": writer=<equipment> stand=<writer stand> alive=<its liveness key> - all three (project_tags)"); break;
            }
            if (!name_ok_kv(alive)) { bad("tag " + t.key + ": alive=" + alive + " - not a KV key"); break; }
            auto ka = tag_alive.find(t.stand);
            if (ka != tag_alive.end() && ka->second != alive) { bad("tag " + t.key + ": stand " + t.stand + " has another alive= key above"); break; }
            for (const auto &o : tag_alive)
                if (o.second == alive && o.first != t.stand) { bad("tag " + t.key + ": alive=" + alive + " is stand " + o.first + "'s"); break; }
            if (!good) break;
            if (!t.out) {
                if (!tag_own_stand.empty() && tag_own_stand != t.stand) { bad("tag " + t.key + ": dir=in of stand " + t.stand + ", the node's other dir=in are of " + tag_own_stand); break; }
                tag_own_stand = t.stand;
            }
            memset(&t.rest, 0, sizeof t.rest);   // HAL default 0: a lost signal reads as off
            if (!safe_s.empty()) {
                if (!t.out) { bad("tag " + t.key + ": safe= on dir=in - only a reader end has a rest value"); break; }
                if (const char *e = safe_of(t.type, safe_s, t.rest)) { bad("tag " + t.key + ": safe=" + safe_s + " - " + e); break; }
            }
            for (const TagEnd &o : tends) {
                if (o.src == t.src && o.sig == t.sig) { bad("tag " + t.key + ": " + t.src + " twice in the tags"); break; }
                if (!o.out && !t.out && o.key == t.key) { bad("tag " + t.key + ": two dir=in ends of one key"); break; }
            }
            for (const HPin &o : hpins) if (o.src == t.src && o.sig == t.sig && o.out && t.out) { bad("tag " + t.key + ": " + t.src + " is a pin dir=out too - one writer"); break; }
            if (!good) break;
            tag_alive[t.stand] = alive;
            if (t.out) tag_alive_of[alive] = t.stand;
            tends.push_back(t);
        } else if (a == "tag_ok") {
            if (w.size() != 2 || (w[1].compare(0, 4, "sig=") && w[1].compare(0, 4, "src=")) || w[1].size() < 5) { bad("tag_ok sig=<signal>|src=<pin>"); break; }
            tag_ok_sig = !w[1].compare(0, 4, "sig="); tag_ok_src = w[1].substr(4);
        } else if (a == "scope_channel" || a == "scope_batch" || a == "scope_dt_ns" || a == "spb_keepalive_s" || a == "hz"
                   || a == "spb_host_ttl_s" || a == "tag_ttl_s") {
            if (w.size() < 2 || !num(w[1], v) || v < 0) { bad(a + ": a number >= 0"); break; }
            if (a == "hz") { if (!(v > 0)) { bad("hz > 0"); break; } hal_period_s = 1.0 / v; }
            else if (a == "scope_channel") s_chan = (int)v;
            else if (a == "scope_batch") s_batch = (int)v;
            else if (a == "scope_dt_ns") s_dt_ns = (long)v;
            else if (a == "spb_host_ttl_s") host_ttl_s = v;
            else if (a == "tag_ttl_s") tag_ttl_s = v;
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
    if (!tends.empty()) {
        if (!(tag_ttl_s > 0)) {
            say(true, "%s: NATS PINS %s: tags without tag_ttl_s (the writer liveness TTL, tag_envelope.ALIVE_TTL_S) - NATS publishing refused\n",
                tag(), path.c_str());
            return false;
        }
        // data before strobes, readers and writers alike (stable: the rest keep the file order)
        std::stable_sort(tends.begin(), tends.end(), [](const TagEnd &x, const TagEnd &y) { return x.strobe < y.strobe; });
    } else if (!tag_ok_src.empty()) {
        say(true, "%s: NATS PINS %s: tag_ok without tags - NATS publishing refused\n", tag(), path.c_str());
        return false;
    }
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

// ---- the PINS set: start and Rebirth (reload) ----

// Everything load_pins fills. A Rebirth parses the file into empty globals and swaps the previous set back
// when the new one is refused - the node never publishes half a file.
struct PinsSet {
    std::vector<HPin> hpins;
    double keepalive_s = 0, hal_period_s = 0, host_ttl_s = 0, tag_ttl_s = 0;
    int s_chan = -1, s_batch = 0;
    long s_dt_ns = 0;
    bool s_float = false;
    std::vector<SCol> scols;
    std::vector<TagEnd> tends;
    std::map<std::string, std::string> tag_alive, tag_alive_of;
    std::string tag_own_stand;
    bool tag_ok_sig = false;
    std::string tag_ok_src;
};
void swap_pins(PinsSet &p)                       // the globals <-> p, field by field
{
    p.hpins.swap(hpins); p.scols.swap(scols); p.tends.swap(tends);
    p.tag_alive.swap(tag_alive); p.tag_alive_of.swap(tag_alive_of);
    p.tag_own_stand.swap(tag_own_stand); p.tag_ok_src.swap(tag_ok_src);
    std::swap(p.keepalive_s, keepalive_s); std::swap(p.hal_period_s, hal_period_s);
    std::swap(p.host_ttl_s, host_ttl_s); std::swap(p.tag_ttl_s, tag_ttl_s);
    std::swap(p.s_chan, s_chan); std::swap(p.s_batch, s_batch); std::swap(p.s_dt_ns, s_dt_ns);
    std::swap(p.s_float, s_float); std::swap(p.tag_ok_sig, tag_ok_sig);
}
bool same_scope(const PinsSet &o)
{
    if (o.s_chan != s_chan || o.s_batch != s_batch || o.s_dt_ns != s_dt_ns || o.s_float != s_float
        || o.scols.size() != scols.size()) return false;
    for (size_t c = 0; c < scols.size(); c++)
        if (o.scols[c].key != scols[c].key || o.scols[c].ch != scols[c].ch || o.scols[c].scale != scols[c].scale) return false;
    return true;
}

// The loaded set against the node: scope needs a HAL component, safe= rows a Primary Host. Sets `guard`.
// false = refused (said).
bool pins_check()
{
    if (s_chan >= 0 && hal_comp < 0) {
        say(true, "%s: NATS PINS scope_channel without a HAL component - publishing refused\n", tag());
        return false;
    }
    size_t n_safe = 0;
    for (const HPin &p : hpins) n_safe += p.has_safe;
    if (n_safe && cfg.host.empty())
        say(true, "%s: NATS PINS: %zu safe= outputs but no Primary Host id (SYNCTWIN_SPB_PRIMARY_HOST, INI [NATS] "
            "PRIMARY_HOST) - they are NOT made safe when the host goes away\n", tag(), n_safe);
    else if (n_safe && !(host_ttl_s > 0)) {
        say(true, "%s: NATS PINS: safe= outputs without spb_host_ttl_s - publishing refused\n", tag());
        return false;
    }
    bool g = n_safe && !cfg.host.empty() && cfg.host.find_first_of(".*> ") == std::string::npos;
    if (n_safe && !cfg.host.empty() && !g) {
        say(true, "%s: NATS Primary Host id '%s' is not a NATS token - publishing refused\n", tag(), cfg.host.c_str());
        return false;
    }
    guard = g;
    return true;
}

// Every subscription of the node on the current set. Called at start and on every Rebirth: a fresh SUB is
// judged by the broker's permissions of NOW (a SUB denied before a permission change stays dead in libnats).
natsStatus subscribe_all()
{
    natsStatus st = natsConnection_Subscribe(&snap_sub, nc, s_snapshot.c_str(), on_snapshot, nullptr);
    if (st == NATS_OK) st = natsConnection_Subscribe(&ncmd_sub, nc, s_ncmd.c_str(), on_ncmd, nullptr);
    bool has_out = false;
    for (const HPin &p : hpins) has_out = has_out || p.out;
    if (st == NATS_OK && has_out) st = natsConnection_Subscribe(&dcmd_sub, nc, s_dcmd_hal.c_str(), on_dcmd, nullptr);
    // tags: watch the republish of every liveness key and reader key (libnats resubscribes on reconnect)
    std::set<std::string> watch;
    for (const auto &a : tag_alive_of) watch.insert(a.first);
    for (const TagEnd &t : tends) if (t.out) watch.insert(t.key);
    for (const std::string &k : watch) {
        if (st != NATS_OK) break;
        tag_subs.push_back(nullptr);
        st = natsConnection_Subscribe(&tag_subs.back(), nc, ("kv.CELL." + k).c_str(), on_tag_msg, nullptr);
    }
    if (st == NATS_OK && guard)
        st = natsConnection_Subscribe(&state_sub, nc, ("spBv1//0.STATE." + cfg.host).c_str(), on_state, nullptr);
    if (own) {
        own_dcmd_subs.assign(own_dcmd_dev.size(), nullptr);
        for (size_t k = 0; k < own_dcmd_dev.size() && st == NATS_OK; k++) {
            std::string subj = "spBv1//0." + cfg.group + ".DCMD." + cfg.node + "." + own->devices()[own_dcmd_dev[k]];
            st = natsConnection_Subscribe(&own_dcmd_subs[k], nc, subj.c_str(), on_own_dcmd, &own_dcmd_dev[k]);
        }
    }
    return st;
}

// Sparkplug B 3.0 Node Control/Rebirth (and SIGHUP of halnats): the node's definition is the PINS file, so a
// Rebirth re-reads it - a guest attached / detached on the fly gets its rows into NBIRTH + DBIRTH (same bdSeq,
// spec 6.4.x: a Rebirth is births, not a new session), the rows of a removed one are gone. A refused file
// keeps the previous set (said). Subscriptions are made again in both cases. Publisher thread, under mx.
void reload()
{
    PinsSet old;
    swap_pins(old);                              // globals empty, `old` = the set published until now
    bool guard0 = guard;
    std::string why;
    if (!cfg.pins.empty() && !load_pins(cfg.pins)) why = "the file refused (above)";
    else if (!same_scope(old)) why = "scope rows changed - the scope thread holds its FIFO; a new session takes them";
    else if (!pins_check()) why = "refused (above)";
    if (!why.empty()) {
        swap_pins(old);                          // the previous set back; the refused one dropped with `old`
        guard = guard0;
        say(true, "%s: NATS Rebirth: PINS %s not taken: %s - the previous set stays\n", tag(), cfg.pins.c_str(), why.c_str());
    }
    bool conn = natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED;
    // the node's own stand leaves the tags: say it dead now, not after the readers' TTL
    if (why.empty() && conn && tlink && !old.tag_own_stand.empty() && old.tag_own_stand != tag_own_stand)
        kv_put(old.tag_alive[old.tag_own_stand], tag_pack_alive(old.tag_own_stand, false, now_unix(), tag_next_seq()));
    destroy_subs();
    natsStatus st = subscribe_all();
    if (st != NATS_OK)
        say(true, "%s: NATS Rebirth: subscribe: %s - the next Rebirth tries again\n", tag(), natsStatus_GetText(st));
    scope_alias0 = hal_alias0 + hpins.size();
    hal_born = false;
    held_safe = false;                           // host_guard judges again: new safe= rows written safe if the host is away
    rested.clear();                              // tag_expire rests every reader end of a dead stand again (new ends too)
    rejected.clear();
    tag_ok_last = -1;
    if (tends.empty()) tlink = false;
    else if (conn && tlink) {
        // the link stays up (tag_ok does not blink): read current over the resubscribe gap, BIRTH the writers again
        if (!tag_birth()) { tlink = false; tag_start = true; }
    } else tag_start = true;
    need_birth = true;
    if (why.empty())
        say(false, "%s: NATS Rebirth: PINS %s re-read - %zu hal (was %zu), %zu tags (was %zu); births follow\n",
            tag(), cfg.pins.c_str(), hpins.size(), old.hpins.size(), tends.size(), old.tends.size());
}

} // namespace

int configure(Config &c, const char *inifile, const char *node_suffix, double default_period)
{
    c.url = env("SYNCTWIN_NATS_URL");
    c.user = env("SYNCTWIN_NATS_USER");
    c.pass = env("SYNCTWIN_NATS_PASS");
    c.group = env("SYNCTWIN_BOX_ID");
    c.node = env("NODE_ID");
    c.host = env("SYNCTWIN_SPB_PRIMARY_HOST");
    if (!c.node.empty() && node_suffix) c.node += node_suffix;
    c.period_s = default_period;
    if (inifile) {
        IniFile ini(inifile);
        if (ini) {
            const char *keys[] = {"URL", "USER", "PASS", "GROUP", "NODE", "PRIMARY_HOST"};
            std::string *dst[] = {&c.url, &c.user, &c.pass, &c.group, &c.node, &c.host};
            for (int k = 0; k < 6; k++)
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
    hal_comp = hal_comp_id;
    // publisher period: INI [NATS] PUBLISH_PERIOD / the owner's default, else the PINS file's hz
    if (!(cfg.period_s > 0)) cfg.period_s = hal_period_s;
    if (!(cfg.period_s > 0)) {
        say(true, "%s: NATS: no publisher period - INI [NATS] PUBLISH_PERIOD or PINS hz - publishing refused\n", tag());
        return -1;
    }
    if (!pins_check()) return -1;
    hal_alias0 = (own ? own->aliases() : 0) + 1;
    scope_alias0 = hal_alias0 + hpins.size();

    const std::string base = "spBv1//0." + cfg.group + ".";
    s_nbirth = base + "NBIRTH." + cfg.node;
    s_ndeath = base + "NDEATH." + cfg.node;
    s_ndata = base + "NDATA." + cfg.node;
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
    if (st == NATS_OK) st = natsOptions_SetErrorHandler(opts, on_async_error, &errw);
    if (st == NATS_OK) st = natsOptions_SetDisconnectedCB(opts, on_disconnected, nullptr);
    if (st == NATS_OK) {
        st = natsConnection_Connect(&nc, opts);
        if (st == NATS_NOT_YET_CONNECTED) st = NATS_OK;
    }
    tag_seq = (int64_t)(now_unix() * 1e6);       // tag_envelope.Sequence: a restarted writer is newer
    std::string own_cmds;
    if (own) {
        own_dcmd_dev.clear();
        for (size_t d = 0; d < own->devices().size(); d++)
            if (own->takes_commands(d)) {
                own_dcmd_dev.push_back(d);
                own_cmds += (own_cmds.empty() ? "" : ", ") + base + "DCMD." + cfg.node + "." + own->devices()[d];
            }
    }
    if (st == NATS_OK) st = subscribe_all();
    natsOptions_Destroy(opts);
    if (st != NATS_OK) {
        say(true, "%s: NATS %s: %s - publishing refused\n", tag(), cfg.url.c_str(), natsStatus_GetText(st));
        destroy_subs();
        natsConnection_Destroy(nc); nc = nullptr;
        return -1;
    }
    if (natsConnection_Status(nc) == NATS_CONN_STATUS_CONNECTED) { need_birth = true; tag_start = true; }
    enabled = true;
    running = true;
    pub_thread = std::thread(publisher);
    if (s_chan >= 0) scope_thread = std::thread(scope_main);
    for (const HPin &p : hpins)
        if (p.out) say(false, "%s: NATS hal out %s (%s %s) <- %s%s\n", tag(), p.metric.c_str(), p.sig ? "signal" : "pin",
                       p.src.c_str(), s_dcmd_hal.c_str(), p.has_safe ? ", safe= while the Primary Host is away" : "");
    if (guard) say(false, "%s: NATS Primary Host %s: STATE spBv1//0.STATE.%s, silence %g s = offline\n",
                   tag(), cfg.host.c_str(), cfg.host.c_str(), host_ttl_s);
    if (!own_cmds.empty()) say(false, "%s: NATS commands <- %s\n", tag(), own_cmds.c_str());
    for (const TagEnd &t : tends)
        say(false, "%s: NATS tag %s %s %s %s %s (writer %s, stand %s)%s\n", tag(), tag_type_names[t.tt], t.key.c_str(),
            t.out ? "->" : "<-", t.sig ? "signal" : "pin", t.src.c_str(), t.writer.c_str(), t.stand.c_str(), t.strobe ? " strobe" : "");
    if (!tends.empty()) say(false, "%s: NATS tags: KV CELL, writer TTL %g s%s%s\n", tag(), tag_ttl_s,
                            tag_own_stand.empty() ? "" : (", alive " + tag_alive[tag_own_stand]).c_str(),
                            tag_ok_src.empty() ? "" : (", tag_ok -> " + tag_ok_src).c_str());
    say(false, "%s: NATS %s%s as Sparkplug node %s/%s, %zu own metrics, %zu hal, %zu scope columns, snapshot on %s\n",
        tag(), cfg.url.c_str(), cfg.user.empty() ? "" : " (user)", cfg.group.c_str(), cfg.node.c_str(),
        own ? own->aliases() : (size_t)0, hpins.size(), scols.size(), s_snapshot.c_str());
    return 0;
}

void rebirth() { need_reload = true; }       // a lock-free atomic store: async-signal-safe

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
            if (tlink) tag_keepalive(false);         // a clean exit of the tag writer: alive false (NDEATH)
            send(s_ndeath, b, false);                // NDEATH carries no seq (6.4.5)
            // ПРЕДПОЛОЖЕНИЕ, не замер: 1 s for NDEATH to leave at shutdown
            natsConnection_FlushTimeout(nc, 1000);
        }
    }
    destroy_subs();
    natsConnection_Destroy(nc); nc = nullptr;
}

} // namespace halnats
