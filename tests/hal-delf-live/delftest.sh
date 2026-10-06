#!/bin/bash
# Hold the realtime thread inside a function and delf the function from
# another process. delf must not return before the thread has left the
# function. Then unloadrt the component, load it again and check that the
# thread runs the new function.
#
# No wall-clock timeouts here: every wait is on a HAL pin. test.sh bounds
# the whole test. (2.9: the python hal module has no get_p and threads
# have no threadbeat pin, so this uses halcmd and the component's own
# pass counter.)
set -e

# wait until pin $1 advanced by $2
wait_count() {
    local start
    start=$(halcmd getp "$1")
    while [ $(( $(halcmd getp "$1") - start )) -lt "$2" ]; do sleep 0.001; done
}

halcmd loadrt delfvictim names=v
halcmd addf v t
wait_count v.count 2

halcmd setp v.hold 1
while [ "$(halcmd getp v.inside)" != TRUE ]; do sleep 0.001; done
# The thread is inside v and stays there for 500 periods. delf must
# wait for it to leave; once delf has returned, v.inside must be false.
halcmd delf v t
if [ "$(halcmd getp v.inside)" = TRUE ]; then
    echo "FAIL: delf returned while the thread was inside v"
    exit 1
fi
echo "ok: delf waits for the thread to leave the function"

halcmd unloadrt delfvictim
halcmd loadrt delfvictim names=w
halcmd addf w t
wait_count w.count 2
echo "ok: the thread runs after delf and unloadrt"
