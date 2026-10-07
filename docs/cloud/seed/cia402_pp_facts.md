# Seed: CiA402 Profile Position facts available to this task

Primary standard texts (IEC 61800-7-201, ETG.6010) are NOT in this repo or container.
Only sources below are given. A claim about the standard may be marked "confirmed" only
with a quote from a source that is actually readable in the container (a public vendor
manual, public ETG/CiA text found online, code comments); otherwise mark it
`NOT CONFIRMED BY PRIMARY SOURCE`.

## Vendor manual excerpt (Inovance servo manual, RU p.245), Profile Position controlword
- bit 4  New set-point: edge 0->1 accepts a new target (0x607A, 0x6081, 0x6083, 0x6084);
         1->0 clears statusword bit 12 (set-point acknowledge).
- bit 5  Change set immediately: 0 - the new target is executed after the current one;
         1 - at once (the running move is replaced).
- bit 6  abs/rel: 0 - 0x607A absolute, 1 - relative.
- bit 8  Halt.

## Where the code is
- `src/hal/components/cia402.comp` exists on branch `synctwin/2.10` of THIS repo
  (`git show origin/synctwin/2.10:src/hal/components/cia402.comp`). Not on cloud/mc-axis-v1.
  Its pp-mode: pins pp-execute (rising edge = new set-point), pp-velocity, pp-accel,
  pp-halt, pp-done, stat-setpoint-ack; in pp_state 1 it sets controlword bit 5 together
  with bit 4 (comment "change set immediately"), bit 5 stays set in state 2.
  Read it fully before concluding what exists and what is missing.
- `mc_axis.comp` drive-profile = 1 hands over the whole move once (pos-cmd, move-velocity,
  move-acceleration, one period of move-execute); done from drv-move-done (= cia402 pp-done).
- Public fork of the EtherCAT driver: https://github.com/SyncTwin/linuxcnc-ethercat
  (a clone from the container is allowed read-only; do not push there).
