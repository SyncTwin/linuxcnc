#!/bin/sh
# exit 0 when thread 't' is still executing passes (siggen.0.update runs in
# it, so its sawtooth output keeps moving)
a=$(halcmd getp siggen.0.sawtooth)
sleep 0.1
b=$(halcmd getp siggen.0.sawtooth)
echo "sawtooth $a -> $b"
[ "$a" != "$b" ]
