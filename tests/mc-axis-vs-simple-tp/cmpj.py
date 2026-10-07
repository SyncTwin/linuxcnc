#!/usr/bin/env python3
# mc_axis track mode with a jerk limit against simple_tp with max_jerk > 0.
# simple_tp.comp has no jerk input; the planner that has one is the jog
# planner of motion, src/emc/motion/simple_tp.c (S-curve, planner type 1),
# built by test.sh as ./ref_simple_tp.  mc_axis (wired as in "REPLACING
# SIMPLE_TP" of mc_axis(9), maxjerk on its jerk input) runs in HAL; sampler
# records its inputs and outputs every period; the recorded inputs are then
# replayed period by period through simple_tp.c.  Checked on the record:
#  - mc_axis keeps maxvel, maxaccel and maxjerk (from its velocity record);
#  - mc_axis never passes a standing target and comes to rest exactly on it;
#  - |mc_axis - simple_tp.c| position <= TOL in every period (see TOL).
# Every wait is on a HAL state; test.sh bounds the whole test.

import hal
import subprocess
import sys
import threading
import time

DT = 0.001          # thread period (cmpj.hal, setexact)
VMAX, AMAX, JMAX = 50.0, 200.0, 5000.0
# halsampler prints 6 decimals: a position or velocity is off by up to 0.5e-6.
EPS_P = 1e-5
EPS_V = 1e-5
# acceleration from two velocities: 2 * 0.5e-6 / DT; jerk from three: 4 * 0.5e-6 / DT^2
EPS_A = 1e-6 / DT
EPS_J = 2e-6 / DT ** 2
# TOL: the two planners follow the same law (velocity rounded off at the
# rate a^2 = 2 J dv, acceleration capped, the stop distance checked every
# period) but discretise it differently, and each difference is a shift in
# time of the same profile, so it costs at most maxvel * that shift:
#  - integration: mc_axis takes the period's new acceleration for the whole
#    period and moves by the mean velocity; simple_tp.c moves by the mean
#    acceleration and the cubic: mc_axis runs half a period ahead (DT/2);
#  - the brake starts in a whole period: simple_tp.c checks the continuous
#    stopping distance (stoppingDist), mc_axis the stop it runs period by
#    period, so the stop can begin one period apart (DT).
# TOL = maxvel * 1.5 DT.  Not compared: maxvel lowered during a move (simple_tp.c
# clips the velocity in one period, an acceleration of (dv / DT), not a ramp).
TOL = VMAX * 1.5 * DT

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
    hal.set_s("step", str(t - g("ramp.out")))


rest_phases = set()   # phases that end at rest on their target (at_rest)


def at_rest():
    rest_phases.add(int(hal.get_s("phase")))
    beat(3)
    wait(lambda: g("mj.in-position") or g("mj.error"))
    beat(200)   # simple_tp.c settles on its own; give it the same periods


hal.set_s("maxvel", str(VMAX))
hal.set_s("maxaccel", str(AMAX))
hal.set_s("maxjerk", str(JMAX))
target(0)
hal.set_p("mj.power", "1")
wait(lambda: g("mj.state") == 1)
phase(1, "enable at rest")
hal.set_s("en", "1")
at_rest()
if g("mj.error"):
    sys.exit("FAIL: mc_axis refused the jerk in track mode (error-id %d)" % g("mj.error-id"))

phase(2, "step 0 -> 100")
target(100); at_rest()
phase(3, "step 100 -> -50")
target(-50); at_rest()

phase(4, "reversal: target 100, then -30 while moving up")
target(100)
wait(lambda: g("mj.pos-cmd") > 0)
phase(5, "reversal: target -30")
target(-30); at_rest()

phase(6, "nearer target while moving: 100, then 70 at 60")
target(100)
wait(lambda: g("mj.pos-cmd") > 60)
phase(7, "nearer target 70")
target(70); at_rest()

phase(8, "enable 0 mid-move")
target(-100)
wait(lambda: g("mj.pos-cmd") < 20)
phase(9, "enable 0")
hal.set_s("en", "0")
beat(3)
wait(lambda: g("mj.vel-cmd") == 0 and not g("mj.busy"))
beat(200)
phase(10, "enable 1 again")
hal.set_s("en", "1"); at_rest()

phase(11, "moving target, 20 u/s")
hal.set_p("ramp.in", "20")
wait(lambda: g("mj.pos-cmd") > -60)
phase(12, "moving target stops")
hal.set_p("ramp.in", "0"); at_rest()

phase(13, "maxjerk 1000 (acceleration never reaches maxaccel), step +30")
hal.set_s("maxjerk", "1000")
beat(3)
target(g("mj.pos-cmd") + 30); at_rest()
phase(14, "step -0.001 (a short S-curve)")
target(g("mj.pos-cmd") - 0.001); at_rest()
hal.set_s("maxjerk", str(JMAX))
beat(3)
phase(15, "step +0.00001 (less than one period at full jerk)")
target(g("mj.pos-cmd") + 0.00001); at_rest()
phase(16, "step +0.000017 (the last period of the ramp lands on the velocity)")
target(g("mj.pos-cmd") + 0.000017); at_rest()

phase(99, "end")
beat(5)
wait(lambda: rows and rows[-1][-1] == "99")
beat(5)
proc.terminate()
over = g("sampler.0.overruns")
hal.set_p("mj.power", "0")

# ---- the record: sample, target en maxvel maxaccel maxjerk pos vel inpos busy phase
cols = [r for r in rows if len(r) == 11]
fails = []


def fail(msg):
    fails.append(msg)
    print("FAIL: " + msg)


if over != 0 or any(int(cols[i][0]) != int(cols[0][0]) + i for i in range(len(cols))):
    fail("sampler lost periods (overruns %d)" % over)

T = [float(c[1]) for c in cols]
EN = [c[2] == "1" for c in cols]
VM = [float(c[3]) for c in cols]
AM = [float(c[4]) for c in cols]
JM = [float(c[5]) for c in cols]
P = [float(c[6]) for c in cols]
V = [float(c[7]) for c in cols]
PH = [int(c[10]) for c in cols]


def where(k):
    return "period %d, phase %d (%s)" % (k, PH[k], phase_names.get(PH[k], "?"))


# simple_tp.c on the same inputs
ref_in = "".join("%s %d %s %s %s\n" % (c[1], c[2] == "1", c[3], c[4], c[5]) for c in cols)
ref = subprocess.run(["./ref_simple_tp"], input=ref_in, capture_output=True, text=True, check=True)
R = [float(line.split()[0]) for line in ref.stdout.splitlines()]
if len(R) != len(cols):
    fail("simple_tp.c replay gave %d periods for %d" % (len(R), len(cols)))

# limits of mc_axis
peak = [0.0, 0.0, 0.0]
for k in range(2, len(cols)):
    a1, a0 = (V[k] - V[k - 1]) / DT, (V[k - 1] - V[k - 2]) / DT
    j = (a1 - a0) / DT
    if abs(V[k]) > VM[k] + EPS_V:
        fail("velocity %.6f over maxvel %.3f at %s" % (V[k], VM[k], where(k))); break
    if abs(a1) > AM[k] + EPS_A:
        fail("acceleration %.3f over maxaccel %.3f at %s" % (a1, AM[k], where(k))); break
    if abs(j) > max(JM[k], JM[k - 1], JM[k - 2]) + EPS_J:
        fail("jerk %.1f over maxjerk %.1f at %s" % (j, JM[k], where(k))); break
    peak = [max(peak[0], abs(V[k])), max(peak[1], abs(a1)), max(peak[2], abs(j))]
print("mc_axis: peak |v| %.4f, |a| %.3f, |j| %.1f" % tuple(peak))

# no overshoot of a standing target, and at rest on it at the end of the run
runs, s = [], 0
for k in range(1, len(cols) + 1):
    if k == len(cols) or T[k] != T[s] or EN[k] != EN[s]:
        runs.append((s, k))
        s = k
for s, e in runs:
    side = 0
    for k in range(s, e):
        d = P[k] - T[s]
        if abs(d) > EPS_P:
            if side and (d > 0) != (side > 0):
                fail("passed the standing target %.6f (at %.6f) at %s" % (T[s], P[k], where(k)))
                break
            side = 1 if d > 0 else -1
    if (EN[s] and PH[e - 1] in rest_phases and (e == len(cols) or PH[e] != PH[e - 1])
            and not (V[e - 1] == 0 and abs(P[e - 1] - T[s]) <= EPS_P)):
        fail("not at rest on the target %.6f at the end of %s" % (T[s], where(e - 1)))

# against simple_tp.c
dev, kd = 0.0, 0
for k in range(min(len(R), len(P))):
    if abs(P[k] - R[k]) > dev:
        dev, kd = abs(P[k] - R[k]), k
print("largest |mc_axis - simple_tp.c| position difference: %.6f at %s (TOL %.6f, periods %d)"
      % (dev, where(kd), TOL, len(cols)))
if dev > TOL:
    fail("mc_axis and simple_tp.c differ by %.6f > TOL %.6f at %s" % (dev, TOL, where(kd)))

if fails:
    sys.exit("%d failure(s)" % len(fails))
print("ok: mc_axis track mode with a jerk limit does what simple_tp.c does with max_jerk")
