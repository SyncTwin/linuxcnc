#!/usr/bin/env python3
# Drive mc_axis through its PLCopen commands on an ideal drive and check
# the outputs. Every wait is on a HAL state; test.sh bounds the test.

import hal
import sys
import time

DISABLED, STANDSTILL, DISCRETE, CONTINUOUS, ERRORSTOP = 0, 1, 3, 4, 7


def g(pin, ax="ax"):
    return hal.get_p(ax + "." + pin)


def s(pin, value, ax="ax"):
    hal.set_p(ax + "." + pin, str(value))


def wait(cond):
    while not cond():
        time.sleep(0.001)


def check(cond, msg):
    if not cond:
        sys.exit("FAIL: %s (state=%d error-id=%d axis-error-id=%d pos=%.6f)"
                 % (msg, g("state"), g("error-id"), g("axis-error-id"), g("actual-position")))
    print("ok: " + msg)


def edge(pin, ax="ax"):
    """Rising edge on a command input; returns the command's number."""
    old = g("command-id", ax)
    s(pin, 1, ax)
    wait(lambda: g("command-id", ax) != old)
    return g("command-id", ax)


def beat(n=2):
    """Wait until the thread has run n more times (one full pass at n=2)."""
    b = hal.get_p("t.threadbeat")
    wait(lambda: hal.get_p("t.threadbeat") - b >= n)


def release(pin, ax="ax"):
    """Drop a command input and let the axis see it low, so that the next
    edge on it is a new edge."""
    s(pin, 0, ax)
    beat()


def move(mode, position=0.0, velocity=50.0):
    s("mode", mode)
    s("position", position)
    s("velocity", velocity)
    return edge("execute")


check(g("state") == DISABLED, "unpowered axis is Disabled")
s("power", 1)
wait(lambda: g("state") == STANDSTILL)
check(True, "MC_Power: Standstill")

# MC_MoveAbsolute to 100
cid = move(0, 100)
vmax = 0.0
while g("done-id") != cid:
    vmax = max(vmax, abs(g("vel-cmd")))
    time.sleep(0.001)
check(g("done") and not g("busy") and not g("error"), "MoveAbsolute 100: Done")
check(abs(g("actual-position") - 100) < 1e-9, "MoveAbsolute 100: at 100")
check(vmax <= 50 + 1e-9, "MoveAbsolute 100: velocity within maxv")
check(g("state") == STANDSTILL, "MoveAbsolute 100: Standstill")
release("execute")
wait(lambda: not g("done"))
check(True, "Done reset after execute fell")

# MC_MoveAbsolute to 0, interrupted by MC_Halt
mid = move(0, 0)
wait(lambda: g("actual-position") < 70)
hid = edge("halt")
wait(lambda: g("done-id") == hid)
check(g("aborted-id") == mid, "Halt: the move is CommandAborted")
check(g("done") and not g("error"), "Halt: Done")
check(g("state") == STANDSTILL and g("vel-cmd") == 0, "Halt: Standstill")
stopped = g("actual-position")
check(0 < stopped < 70, "Halt: stopped short of the target")
release("execute"); release("halt")

# a target outside the soft limits is refused, no motion
rid = move(0, 200)
wait(lambda: g("failed-id") == rid)
check(g("error") and g("error-id") == 5 and g("axis-error-id") == 0, "target outside limits: error-id 5")
check(g("state") == STANDSTILL and g("actual-position") == stopped, "refused move: no motion")
release("execute")
wait(lambda: not g("error"))
check(True, "refusal Error reset after execute fell")

# MC_SetPosition: the axis frame moves, the drive does not
fb = g("pos-fb")
s("mode", 0); s("position", 10)
sid = edge("set-position")
wait(lambda: g("done-id") == sid)
check(abs(g("actual-position") - 10) < 1e-9 and g("pos-fb") == fb, "SetPosition 10: no motion")
release("set-position")

# MC_MoveRelative +5
cid = move(1, 5)
wait(lambda: g("done-id") == cid)
check(abs(g("actual-position") - 15) < 1e-9, "MoveRelative +5: at 15")
release("execute")

# MC_MoveVelocity -20, then MC_Halt
cid = move(2, velocity=-20)
wait(lambda: g("in-velocity"))
check(g("state") == CONTINUOUS and abs(g("vel-cmd") + 20) < 1e-9, "MoveVelocity -20: InVelocity")
hid = edge("halt")
wait(lambda: g("done-id") == hid)
check(g("aborted-id") == cid and g("state") == STANDSTILL, "MoveVelocity halted")
release("execute"); release("halt")

# ContinuousUpdate (PLCopen Part 1 v2.0, 2.4.6), the standard's own example:
# a MoveRelative takes a new Distance and Velocity while it runs, without a
# new Execute edge, and ends Done at the new distance from where it started
cid = move(0, 0)
wait(lambda: g("done-id") == cid)
release("execute")
s("continuous-update", 1)
cid = move(1, 60, velocity=20)
wait(lambda: g("actual-position") >= 30)
s("position", 90); s("velocity", 40)
vmax = 0.0
while g("done-id") != cid:
    vmax = max(vmax, abs(g("vel-cmd")))
    time.sleep(0.001)
check(abs(g("actual-position") - 90) < 1e-9, "ContinuousUpdate: relative move ends at the updated distance")
check(20 + 1e-9 < vmax <= 40 + 1e-9, "ContinuousUpdate: the updated velocity is taken")
check(g("command-id") == cid and g("aborted-id") != cid, "ContinuousUpdate: same command, not aborted")
s("position", 0)
beat(10)
check(g("done") and g("actual-position") == 90 and g("state") == STANDSTILL, "ContinuousUpdate: no new values after Done")
release("execute")

# ContinuousUpdate is taken at the Execute edge: set later, it changes nothing
s("continuous-update", 0)
cid = move(0, 50, velocity=20)
wait(lambda: g("actual-position") < 70)
s("continuous-update", 1); s("position", 10)
wait(lambda: g("done-id") == cid)
check(abs(g("actual-position") - 50) < 1e-9, "ContinuousUpdate 0 at the edge: change ignored")
release("execute")

# ContinuousUpdate falling ends the update for the rest of the move
cid = move(0, 10, velocity=20)
wait(lambda: g("actual-position") < 40)
s("continuous-update", 0); s("position", 0)
wait(lambda: g("done-id") == cid)
check(abs(g("actual-position") - 10) < 1e-9, "ContinuousUpdate falling: the move keeps its target")
release("execute")

# an updated target outside the soft limits ends the move with error-id 5
s("continuous-update", 1)
cid = move(0, 60, velocity=20)
wait(lambda: g("actual-position") > 20)
s("position", 200)
wait(lambda: g("failed-id") == cid)
check(g("error") and g("error-id") == 5 and not g("busy"), "ContinuousUpdate outside limits: error-id 5")
wait(lambda: g("state") == STANDSTILL)
check(20 < g("actual-position") < 60, "ContinuousUpdate outside limits: halted")
release("execute"); s("continuous-update", 0)
wait(lambda: not g("error"))

# following a target that moves every period (the simple_tp use): fx.position
# is ramp.out, which rises at ramp.in units/s inside the thread
s("power", 1, "fx")
wait(lambda: g("state", "fx") == STANDSTILL)
s("mode", 0, "fx"); s("continuous-update", 1, "fx")
hal.set_p("ramp.in", "20")
beat(3)
fid = edge("execute", "fx")
wait(lambda: g("actual-position", "fx") >= 40)
check(g("busy", "fx") and g("state", "fx") == DISCRETE and g("command-id", "fx") == fid,
      "follow: one command follows the moving target")
check(abs(g("vel-cmd", "fx") - 20) < 0.5, "follow: at the target's velocity")
lag = hal.get_p("ramp.out") - g("pos-cmd", "fx")
check(0 < lag < 2, "follow: lags the target by about v^2/2a (%.3f)" % lag)
hal.set_p("ramp.in", "0")
wait(lambda: g("done-id", "fx") == fid)
check(g("pos-cmd", "fx") == hal.get_p("ramp.out") and g("aborted-id", "fx") != fid,
      "follow: Done at the target where it stopped")
held = g("pos-cmd", "fx")
hal.set_p("ramp.in", "20")
beat(50)
hal.set_p("ramp.in", "0")
check(g("pos-cmd", "fx") == held, "follow: after Done a moving target needs a new Execute edge")
release("execute", "fx")
s("power", 0, "fx")

# drive fault: ErrorStop, then MC_Reset
s("drv-fault", 1)
wait(lambda: g("state") == ERRORSTOP)
check(g("error-id") == 3 and g("axis-error-id") == 3, "drive fault: ErrorStop, error-id 3")
cid = move(0, 0)
wait(lambda: g("failed-id") == cid)
check(g("state") == ERRORSTOP, "move refused in ErrorStop")
release("execute")
s("drv-fault", 0)
rid = edge("reset")
wait(lambda: g("state") == STANDSTILL)
check(not g("error") and g("axis-error-id") == 0, "MC_Reset: Standstill")
release("reset")

s("power", 0)
wait(lambda: g("state") == DISABLED)
check(True, "power off: Disabled")
