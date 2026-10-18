/********************************************************************
* Description: halnats_err.hh
*   The async error handler of the halnats publisher (natsOptions_SetErrorHandler).
*   The broker drops a publish the user has no right for and says so only through
*   this callback ("Permissions Violation for Publish to ..."); without it the
*   node looked healthy and published into the void. A line per kind of error
*   (the key = operation + subject) at the 1st, 10th, 100th ... occurrence with the
*   count, not one per publish; the totals and the last error go to the node's
*   NBIRTH/NDATA as publisher/errors and publisher/last_error.
*   Needs only libnats and say() of halnats.hh.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#ifndef HALNATS_ERR_HH
#define HALNATS_ERR_HH

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <string.h>
#include <nats/nats.h>

#include "halnats.hh"

namespace halnats {

class ErrWatch {
public:
    // one error of kind `key`; the log line `msg` comes out when the count is 1, 10, 100 ...
    void note(const std::string &key, const std::string &msg)
    {
        uint64_t n;
        {
            std::lock_guard<std::mutex> g(mx_);
            n = ++per_[key];
            total_++;
            last_ = msg;
        }
        for (uint64_t t = 1; t <= n; t *= 10)
            if (t == n) { say(true, "halnats: %s (x%llu)\n", msg.c_str(), (unsigned long long)n); break; }
    }
    uint64_t total() const { std::lock_guard<std::mutex> g(mx_); return total_; }
    // SUBs the broker refused so far: a Rebirth resubscribes only when this moved (permissions may have changed)
    void sub_denied() { sub_denied_++; }
    uint64_t sub_denied_count() const { return sub_denied_; }
    std::string last() const { std::lock_guard<std::mutex> g(mx_); return last_; }

private:
    mutable std::mutex mx_;
    std::map<std::string, uint64_t> per_;
    uint64_t total_ = 0;
    std::string last_;
    std::atomic<uint64_t> sub_denied_{0};
};

// text of the broker's -ERR: `Permissions Violation for Publish to "subj"` (or Subscription to)
inline std::string perm_text(const std::string &t)
{
    static const struct { const char *needle, *what; } k[] = {
        {"Permissions Violation for Publish to ", "нет прав публиковать "},
        {"Permissions Violation for Subscription to ", "нет прав подписаться на "},
    };
    for (const auto &e : k) {
        size_t p = t.find(e.needle);
        if (p == std::string::npos) continue;
        std::string s = t.substr(p + strlen(e.needle));
        while (!s.empty() && (s.front() == '"' || s.front() == '\'')) s.erase(0, 1);
        while (!s.empty() && (s.back() == '"' || s.back() == '\'')) s.pop_back();
        return std::string(e.what) + s;
    }
    return std::string();
}

// the natsErrHandler; closure = ErrWatch*
inline void on_async_error(natsConnection *c, natsSubscription *sub, natsStatus err, void *closure)
{
    ErrWatch *w = static_cast<ErrWatch *>(closure);
    const char *txt = nullptr;
    if (c) natsConnection_GetLastError(c, &txt);
    std::string t = txt && *txt ? txt : natsStatus_GetText(err);
    std::string perm = perm_text(t);
    if (t.find("Permissions Violation for Subscription to ") != std::string::npos) w->sub_denied();
    if (!perm.empty()) { w->note(perm, perm); return; }
    if (err == NATS_SLOW_CONSUMER) {
        const char *s = sub ? natsSubscription_GetSubject(sub) : nullptr;
        std::string m = std::string("медленный потребитель (сообщения теряются): ") + (s ? s : "?");
        w->note(m, m);
        return;
    }
    std::string m = "ошибка NATS: " + t;
    w->note(m, m);
}

} // namespace halnats

#endif
