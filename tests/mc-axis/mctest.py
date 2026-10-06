#!/usr/bin/env python3
# Drive mc_axis through its PLCopen commands on an ideal drive and check
# the outputs. Every wait is on a HAL state; test.sh bounds the test.

import hal
import sys
import time

DISABLED, STANDSTILL, DISCRETE, CONTINUOUS, ERRORSTOP = 0, 1, 3, 4, 7


def g(pin):
    return hal.get_p("ax." + pin)


def s(pin, value):
    hal.set_p("ax." + pin, str(value))


def wait(cond):
    while not cond():
        time.sleep(0.001)


def check(cond, msg):
    if not cond:
        sys.exit("FAIL: %s (state=%d error-id=%d axis-error-id=%d pos=%.6f)"
                 % (msg, g("state"), g("error-id"), g("axis-error-id"), g("actual-position")))
    print("ok: " + msg)


def edge(pin):
    """Rising edge on a command input; returns the command's number."""
    old = g("command-id")
    s(pin, 1)
    wait(lambda: g("command-id") != old)
    return g("command-id")


def beat(n=2):
    """Wait until the thread has run n more times (one full pass at n=2)."""
    b = hal.get_p("t.threadbeat")
    wait(lambda: hal.get_p("t.threadbeat") - b >= n)


def release(pin):
    """Drop a command input and let the axis see it low, so that the next
    edge on it is a new edge."""
    s(pin, 0)
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
