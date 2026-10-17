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
*   Config: INI [NATS] URL, GROUP, NODE (PUBLISH_PERIOD optional, default
*   [TASK] CYCLE_TIME). No URL - publishing off, said once in the log.
*
* License: GPL Version 2
* Copyright (c) 2026 SyncTwin Technologies
********************************************************************/
#ifndef TASKNATS_HH
#define TASKNATS_HH

class EMC_STAT;

// 0 = publishing on or deliberately off; -1 = [NATS] present but refused (said in the log)
int taskNatsInit(const char *inifile, int joints, double task_cycle_time);
void taskNatsCycle(const EMC_STAT *stat);
void taskNatsShutdown(void);

#endif
