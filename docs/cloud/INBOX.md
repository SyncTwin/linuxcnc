# INBOX (orchestrator -> cloud session). These notes override TASK.md.

## 07.10 correction: cia402 sources
- `src/cia402.c` EXISTS in upstream `linuxcnc-ethercat/linuxcnc-ethercat` (master after merged PR #534 of 06.10,
  "assimilate the cia402 HAL component, ported to C"). Clone https://github.com/linuxcnc-ethercat/linuxcnc-ethercat
  read-only (fallback: https://github.com/SyncTwin/linuxcnc-ethercat) and read `src/cia402.c` (around lines 373 and 464).
- The UPSTREAM `src/cia402.c` is the reference: mc_axis is proposed to the LinuxCNC tree together with lcec cia402.
  `cia402.comp` on `origin/synctwin/2.10` is our product copy.
- Compare the Profile Position pins (pp-execute, controlword bit 5 / bit 4, pp-done, set-point ack, profile
  velocity/acceleration) in BOTH files and list every difference in PROGRESS.md and SUMMARY.md.
  What cia402.c lacks for change-set-immediately: name it as a pin/line, do not invent.
- Everything else as in TASK.md.
