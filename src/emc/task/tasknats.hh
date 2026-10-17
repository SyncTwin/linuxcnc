/********************************************************************
* Description: tasknats.hh
*   Task status published to NATS as Sparkplug B, next to NML.
*
*   taskNatsCycle() runs in the task main loop right after the NML status
*   write: it diffs the status against its last snapshot and pushes the
*   changes into a single-producer/single-consumer lock-free ring. A separate
*   thread drains the ring and publishes. The task loop never waits on NATS:
*   a full ring counts a drop and the change is retried on the next cycle.
*
*   The node (connection, births, Rebirth, devices hal and scope) is the
*   halnats library (hal/utils/halnats/halnats.hh) - the same code as the
*   halnats binary of VMs without LinuxCNC; the task adds its status devices.
*   Config: the VM environment SYNCTWIN_NATS_URL/USER/PASS, SYNCTWIN_BOX_ID
*   (group), NODE_ID + "-task" (node); INI [NATS] URL USER PASS GROUP NODE
*   override, PUBLISH_PERIOD optional (default [TASK] CYCLE_TIME), PINS = <file>
*   (relative to the INI's folder) - devices hal / scope. No URL - publishing
*   off, said once in the log.
*
*   Commands: DCMD spBv1//0.<group>.DCMD.<node>.task|motion, one command per message =
*   cmd/serial (Int64, growing) + one verb: task  state | mode | mdi | program/open |
*   program/run | program/pause | program/resume | program/step | abort; motion  home | unhome |
*   teleop | jog/stop | jog/continuous | jog/increment (+ jog/joint, jog/velocity, jog/distance).
*   state ESTOP and abort take a priority lane ahead of the queue; a full queue answers ERROR.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#ifndef TASKNATS_HH
#define TASKNATS_HH

class EMC_STAT;
class RCS_CMD_MSG;

// 0 = publishing on or deliberately off; -1 = [NATS] present but refused (said in the log)
// hal_comp_id: the task's HAL component (iocontrol) - the sampler FIFO is attached with it
int taskNatsInit(const char *inifile, int joints, double task_cycle_time, int hal_comp_id);
void taskNatsCycle(const EMC_STAT *stat);
void taskNatsShutdown(void);

// Commands from NATS - the second source next to NML (DCMD of devices task / motion, see tasknats.cc).
// Task main loop, only in a cycle in which NML brought no new command: the next message to plan
// (emcCommand = it), one per cycle, or nullptr. The answer goes out as an event (task cmd/serial,
// cmd/state RCS_DONE/EXEC/ERROR, cmd/error) from taskNatsCycle - no client polling.
RCS_CMD_MSG *taskNatsCommand(void);
// emcOperatorError / emcOperatorText / emcOperatorDisplay (kind 0 / 1 / 2) -> device task
// operator/error|text|display (String), next to the NML error channel.
void taskNatsMessage(int kind, const char *text);

#endif
