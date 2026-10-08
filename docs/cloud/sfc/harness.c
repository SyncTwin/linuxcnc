/* Harness for pulse.st (matiec iec2c output: config.c, res.c, POUS.c/.h).
 * Drives inst (RES__INST) scan by scan, counts action body executions per scan
 * and checks them against the expected counts of each scenario.
 *
 * Scenario (k, s4, m):
 *   scan 0           idle (initial step S0)
 *   scan 1           go=1      -> S1 and S3 activated (S1 entry scan)
 *   scan 1+k         nxt=1     -> S1 left, S4 activated (S1 exit scan); S1 active k scans
 *   scan 1+k+s4      nxt=1     -> S4 left, S2 activated (R on A_S and on A_N3)
 *   scan 1+k+s4+m    back=1    -> S2,S3 left, S0 activated
 *   then 3 idle scans
 * Build: gcc -I<matiec>/lib/C -I<gen> harness.c <gen>/config.c <gen>/res.c
 */
#include <stdio.h>
#include <stdlib.h>
#include "iec_std_lib.h"
#include "accessor.h"
#include "POUS.h"

TIME __CURRENT_TIME;
extern PULSE_data__ RES__INST;
void config_init__(void);
void config_run__(unsigned long tick);

#define G(f) ((long)__GET_VAR(RES__INST.f))

enum { P, P1, P0, N, S, N3, NC };
static const char *names[NC] = { "P", "P1", "P0", "N", "S", "N3" };

static int failures;

static void check(const char *ver, const char *q, const char *scen, const char *what,
                  long expect, long fact)
{
    int ok = (expect == fact);
    if (!ok) failures++;
    printf("ROW|%s|%s|%s|%s|%ld|%ld|%s\n", ver, q, scen, what, expect, fact, ok ? "PASS" : "FAIL");
}

static void fact(const char *ver, const char *q, const char *scen, const char *what, long v)
{
    printf("ROW|%s|%s|%s|%s|-|%ld|FACT\n", ver, q, scen, what, v);
}

static void scenario(const char *ver, int k, int s4, int m)
{
    char scen[64];
    int total = 1 + k + s4 + m + 4, t;
    long prev[NC] = {0}, cur[NC];
    long entry[NC] = {0}, exit_[NC] = {0}, after_exit[NC] = {0}, after_entry[NC] = {0};
    int t_entry = 1, t_exit = 1 + k, t_s2 = 1 + k + s4, t_back = 1 + k + s4 + m;

    snprintf(scen, sizeof scen, "S1 active %d scan%s", k, k == 1 ? "" : "s");
    __CURRENT_TIME.tv_sec = 0;
    __CURRENT_TIME.tv_nsec = 0;
    config_init__();
    printf("== %s k=%d s4=%d m=%d\n", ver, k, s4, m);
    printf("scan go nxt back | S0 S1 S4 S2 S3 | dP dP1 dP0 dN dS dN3\n");
    for (t = 0; t < total; t++) {
        int i;
        __SET_VAR(RES__INST., GO, , t == t_entry);
        __SET_VAR(RES__INST., NXT, , t == t_exit || t == t_s2);
        __SET_VAR(RES__INST., BACK, , t == t_back);
        __CURRENT_TIME.tv_nsec += 10000000;
        if (__CURRENT_TIME.tv_nsec >= 1000000000) { __CURRENT_TIME.tv_sec++; __CURRENT_TIME.tv_nsec -= 1000000000; }
        config_run__(t);
        cur[P] = G(CP); cur[P1] = G(CP1); cur[P0] = G(CP0);
        cur[N] = G(CN); cur[S] = G(CS); cur[N3] = G(CN3);
        printf("%4d %2d %3d %4d | %2ld %2ld %2ld %2ld %2ld |", t, t == t_entry, t == t_exit || t == t_s2,
               t == t_back, G(S0_X), G(S1_X), G(S4_X), G(S2_X), G(S3_X));
        for (i = 0; i < NC; i++) {
            long d = cur[i] - prev[i];
            printf(" %2ld", d);
            if (t == t_entry) entry[i] = d;
            if (t == t_entry + 1 && t_entry + 1 < t_exit) after_entry[i] = d;
            if (t == t_exit) exit_[i] = d;
            if (t == t_exit + 1) after_exit[i] = d;
            prev[i] = cur[i];
        }
        printf("\n");
    }

    /* P, P1: exactly once, in the entry scan; P0: exactly once, in the exit scan */
    check(ver, "P",  scen, "total per activation", 1, cur[P]);
    check(ver, "P",  scen, "in entry scan", 1, entry[P]);
    check(ver, "P1", scen, "total per activation (entry edge)", 1, cur[P1]);
    check(ver, "P1", scen, "in entry scan", 1, entry[P1]);
    check(ver, "P0", scen, "total per deactivation (exit edge)", 1, cur[P0]);
    check(ver, "P0", scen, "in exit scan", 1, exit_[P0]);
    check(ver, "P0", scen, "in scan after exit", 0, after_exit[P0]);
    check(ver, "P1", scen, "in scan after exit", 0, after_exit[P1]);
    if (k > 1) {
        check(ver, "P",  scen, "in 2nd active scan", 0, after_entry[P]);
        check(ver, "P1", scen, "in 2nd active scan", 0, after_entry[P1]);
    }
    /* N: every scan S1 is active (S1.X TRUE) + one final scan (e54dcd9) */
    check(ver, "N", scen, "runs with S1.X=TRUE", k, G(CNA));
    check(ver, "N", scen, "total (k + final scan)", k + 1, cur[N]);
    /* S set in S1, kept through S4, R in S2: Q off from the first S2 scan, one final scan there */
    check(ver, "S", scen, "runs with S1.X=TRUE", k, G(CSA1));
    check(ver, "S", scen, "runs with S4.X=TRUE (latched after S1)", s4, G(CSA4));
    check(ver, "S", scen, "runs with S2.X=TRUE (R active; final scan only)", 1, G(CSA2));
    check(ver, "S", scen, "total", k + s4 + 1, cur[S]);
    /* R on A_N3 in S2 while N on A_N3 in the parallel S3: record the fact */
    fact(ver, "R vs N (other step)", scen, "A_N3 runs while S2 (R) active", G(CN3R));
    fact(ver, "R vs N (other step)", scen, "A_N3 runs with S3.X=TRUE", G(CN3A));
    fact(ver, "R vs N (other step)", scen, "A_N3 total", cur[N3]);
    (void)names;
}

int main(int argc, char **argv)
{
    const char *ver = argc > 1 ? argv[1] : "?";
    scenario(ver, 1, 2, 3);
    scenario(ver, 4, 2, 3);
    printf("FAILURES %d\n", failures);
    return failures ? 1 : 0;
}
