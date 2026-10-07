#!/usr/bin/env python3
# simple_tp and mc_axis (track mode, wired as in "REPLACING SIMPLE_TP" of
# mc_axis(9)) run side by side in one thread on the same inputs; sampler
# records every period of both, and the record is checked:
#  - neither exceeds maxvel (except while ramping down to a lowered maxvel)
#    nor maxaccel;
#  - neither passes a target that stands still (no overshoot), enable = 0
#    included;
#  - where simple_tp comes to rest at a standing target, mc_axis is at rest
#    there too, at most N periods later.
# Every wait is on a HAL state; test.sh bounds the whole test.
# MCTP_MODE=0 runs mc_axis as a PLCopen MoveAbsolute with ContinuousUpdate
# instead of track mode: that run must fail (Done ends the following).

import hal
import os
import subprocess
import sys
import threading
import time

DT = 0.001          # thread period (cmp.hal, setexact)
# N: a stopping profile is discretised differently by the two planners
# (simple_tp: v = -a*dt + sqrt(2*a*d + (a*dt)^2), position += v*dt;
# mc_axis: the same law, position += mean velocity * dt, and a final snap when
# the rest fits in half a period's velocity step).  Each end of a move (start,
# stop) can shift by one period, so 2 periods.
N = 2
EPS_P = 1e-5        # halsampler prints 6 decimals
EPS_V = 1e-5
EPS_DV = 3e-6

MODE = int(os.environ.get("MCTP_MODE", "3"))

rows = []
proc = subprocess.Popen(["halsampler", "-t"], stdout=subprocess.PIPE, text=True)


def reader():
    for line in proc.stdout:
        rows.append(line.split())


threading.Thread(target=reader, daemon=True).start()

phase_names = {}


def phase(n, name):
    phase_names[n] = name
    hal.set_s("phase", str(n))


def g(pin):
    return hal.get_p(pin)


def wait(cond):
    while not cond():
        time.sleep(0.001)


def beat(n=2):
    b = g("t.threadbeat")
    wait(lambda: g("t.threadbeat") - b >= n)


def target(t):
    """Make the target t: the step signal is t minus the standing ramp."""
    hal.set_s("step", str(t - g("ramp.out")))


def at_rest():
    beat(3)
    wait(lambda: not g("simple-tp.0.active")
         and (g("mt.in-position") or not g("mt.busy")))
    beat(3)


hal.set_s("maxvel", "50")
hal.set_s("maxaccel", "200")
target(0)
if MODE != 3:
    hal.set_p("mt.continuous-update", "1")
hal.set_p("mt.mode", str(MODE))
hal.set_p("mt.power", "1")
wait(lambda: g("mt.state") == 1)
phase(1, "enable at rest")
hal.set_s("en", "1")
at_rest()

phase(2, "step 0 -> 100")
target(100); at_rest()
phase(3, "step 100 -> -50")
target(-50); at_rest()

phase(4, "reversal: target 100, then -30 while moving up")
target(100)
wait(lambda: g("simple-tp.0.current-pos") > 0)
phase(5, "reversal: target -30")
target(-30); at_rest()

phase(6, "enable 0 mid-move")
target(100)
wait(lambda: g("simple-tp.0.current-pos") > 20)
phase(7, "enable 0")
hal.set_s("en", "0")
beat(3)
wait(lambda: g("simple-tp.0.current-vel") == 0 and g("mt.vel-cmd") == 0
     and not g("mt.busy"))
beat(3)
phase(8, "enable 1 again")
hal.set_s("en", "1"); at_rest()

phase(9, "maxvel 20 mid-move")
target(-100)
wait(lambda: g("simple-tp.0.current-pos") < 60)
hal.set_s("maxvel", "20")
wait(lambda: g("simple-tp.0.current-pos") < 0)
phase(10, "maxaccel 500, maxvel 80 mid-move")
hal.set_s("maxaccel", "500"); hal.set_s("maxvel", "80")
at_rest()
hal.set_s("maxvel", "50"); hal.set_s("maxaccel", "200")

phase(11, "moving target, 20 u/s")
hal.set_p("ramp.in", "20")
wait(lambda: g("simple-tp.0.current-pos") > -60)
phase(12, "moving target stops")
hal.set_p("ramp.in", "0"); at_rest()

phase(13, "moving target after it was reached, -20 u/s")
hal.set_p("ramp.in", "-20")
wait(lambda: g("simple-tp.0.current-pos") < -80)
phase(14, "moving target stops again")
hal.set_p("ramp.in", "0"); at_rest()

phase(15, "enable 0 at rest, target 40, enable 1")
hal.set_s("en", "0"); beat(3)
target(40); beat(20)
hal.set_s("en", "1"); at_rest()

phase(99, "end")
beat(5)
wait(lambda: rows and rows[-1][-1] == "99")
beat(5)
proc.terminate()
over = g("sampler.0.overruns")

# ---- the record
cols = [(int(r[0]), [float(x) for x in r[1:8]], [r[8] == "1", r[9] == "1", r[10] == "1", r[11] == "1"], int(r[12]))
        for r in rows if len(r) == 13]
fails = []


def fail(msg):
    fails.append(msg)
    print("FAIL: " + msg)


if over != 0 or any(cols[i][0] != cols[0][0] + i for i in range(len(cols))):
    fail("sampler lost periods (overruns %d)" % over)

T = [c[1][0] for c in cols]
VMAX = [c[1][5] for c in cols]
AMAX = [c[1][6] for c in cols]
EN = [c[2][0] for c in cols]
PH = [c[3] for c in cols]
planners = {"simple_tp": ([c[1][1] for c in cols], [c[1][2] for c in cols]),
            "mc_axis": ([c[1][3] for c in cols], [c[1][4] for c in cols])}


def where(k):
    return "period %d, phase %d (%s)" % (k, PH[k], phase_names.get(PH[k], "?"))


for name, (P, V) in planners.items():
    vpeak = apeak = 0.0
    for k in range(1, len(cols)):
        if abs(V[k]) > VMAX[k] + EPS_V and abs(V[k]) >= abs(V[k - 1]):
            fail("%s: velocity %.6f over maxvel %.3f at %s" % (name, V[k], VMAX[k], where(k)))
            break
        if abs(V[k] - V[k - 1]) > max(AMAX[k], AMAX[k - 1]) * DT + EPS_DV:
            fail("%s: acceleration %.3f over maxaccel at %s" % (name, (V[k] - V[k - 1]) / DT, where(k)))
            break
        vpeak = max(vpeak, abs(V[k]))
        apeak = max(apeak, abs(V[k] - V[k - 1]) / DT)
    print("%s: peak |v| %.4f, peak |a| %.3f" % (name, vpeak, apeak))

# runs of a standing target and a constant enable
runs, s = [], 0
for k in range(1, len(cols) + 1):
    if k == len(cols) or T[k] != T[s] or EN[k] != EN[s]:
        runs.append((s, k))
        s = k

for name, (P, V) in planners.items():
    for s, e in runs:
        side = 0
        for k in range(s, e):
            d = P[k] - T[s]
            if abs(d) > EPS_P:
                if side and (d > 0) != (side > 0):
                    fail("%s: passed the standing target %.6f (at %.6f) at %s" % (name, T[s], P[k], where(k)))
                    break
                side = 1 if d > 0 else -1


def rest_from(P, V, s, e):
    """First period from which the planner stands at the target to the end of the run."""
    k = e
    while k > s and V[k - 1] == 0.0 and abs(P[k - 1] - T[s]) <= EPS_P:
        k -= 1
    return k if k < e else None


late, ps, pm = [], planners["simple_tp"], planners["mc_axis"]
for s, e in runs:
    if not EN[s] or e - s < 2:
        continue
    ks = rest_from(*ps, s, e)
    if ks is None:
        continue
    km = rest_from(*pm, s, e)
    if km is None:
        fail("mc_axis: not at the target %.6f where simple_tp came to rest (%s)" % (T[s], where(ks)))
        continue
    late.append(km - ks)
    if km - ks > N:
        fail("mc_axis: at rest %d periods after simple_tp (N = %d) at %s" % (km - ks, N, where(km)))
if late:
    print("arrivals: %d, mc_axis minus simple_tp in periods: min %d, max %d"
          % (len(late), min(late), max(late)))

for s, e in runs:
    if not EN[s] and PH[s] == 7:
        print("enable 0 at %.3f u/s: simple_tp stops after %.6f, mc_axis after %.6f"
              % (ps[1][s], ps[0][e - 1] - ps[0][s], pm[0][e - 1] - pm[0][s]))
dev = max(abs(a - b) for a, b in zip(ps[0], pm[0]))
print("largest |mc_axis - simple_tp| position difference: %.6f (periods recorded: %d)" % (dev, len(cols)))

hal.set_p("mt.power", "0")
if fails:
    sys.exit("%d failure(s)" % len(fails))
print("ok: mc_axis track mode does what simple_tp does")
