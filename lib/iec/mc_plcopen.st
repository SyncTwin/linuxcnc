(* PLCopen Motion Part 1 function blocks for iec2comp(1): interface on ST, body on C ({{ }} lands in POUS.c as is).
   Copyright (C) 2026 SyncTwin <https://github.com/SyncTwin>, yurc. GPL-2.0-or-later, as LinuxCNC.
   iec2comp prepends this file to program.st. An axis is a number (AXIS_REF.AxisNo); the host (iec_mc in the
   generated .comp) turns it into pins of mc-axis.N (mc_axis(9)), the FB never sees HAL. One C entry for every Execute FB:
     iec_mc(axis, kind, rise, Execute, a, b, Acceleration, Deceleration, Jerk, &Id, &St, &Done, &Busy, &Error, &ErrorID,
            &InVelocity, &CommandAborted, &Active)
   kind 0 absolute (a = Position) · 1 relative (a = Distance) · 2 velocity (b signed) · 3 halt ·
   4 set position (a = Position, b <> 0 = Relative; mc-axis.N.set-position) · 5 reset (mc-axis.N.reset is held with Execute — a drive
   fault clears in ms, a one-period edge would lose the request, mc_axis.comp ST_ERROR_STOP).
   Outputs (P1 § 2.4.1): the instance keeps its command number (Id) and its own outcome (St), so Done/Error/
   CommandAborted are its own, not the axis's newest; they stay while Execute is held, are reset by the falling
   edge of Execute, and when Execute fell before the end they are set for ONE cycle.
   Move FBs: Acceleration/Deceleration/Jerk are latched with Position/Velocity on the rising edge of Execute (P1 § 2.4.1)
   and reach mc-axis.N as given; an unnamed input is 0 and mc_axis reads 0 as the axis default (acceleration = maxa,
   deceleration = acceleration, jerk = none — TwinCAT "0 = axis default"; Acceleration/Deceleration/Jerk are optional
   "E" inputs in P1). Active: this instance owns the axis's newest command and the axis is still Busy with it. Busy runs to the end
   whatever Execute does. MC_MoveVelocity: InVelocity instead of Done, Busy stays until aborted.
   ⚠ MATIEC runs no final scan of an N action whose step left (IEC 61131-3 2.6.4.5 asks for one with Q
   FALSE): an FB not called last tick had Execute FALSE — hence LastTick, else re-entry into the same step
   gives no rising edge (proven by a test run). *)
TYPE
  AXIS_REF : STRUCT AxisNo : INT; END_STRUCT;
END_TYPE

FUNCTION_BLOCK MC_MoveAbsolute
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; Position : REAL; Velocity : REAL; Acceleration : REAL; Deceleration : REAL; Jerk : REAL; END_VAR
  VAR_OUTPUT Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_mc(int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                       IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    iec_mc(GetFbVar(AXIS, .AXISNO.value), 0, rise, ex, GetFbVar(POSITION), GetFbVar(VELOCITY),
           GetFbVar(ACCELERATION), GetFbVar(DECELERATION), GetFbVar(JERK),
           &GetFbVar(ID), &GetFbVar(ST), &GetFbVar(DONE), &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &nil, &GetFbVar(COMMANDABORTED), &GetFbVar(ACTIVE));
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK

FUNCTION_BLOCK MC_MoveRelative
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; Distance : REAL; Velocity : REAL; Acceleration : REAL; Deceleration : REAL; Jerk : REAL; END_VAR
  VAR_OUTPUT Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_mc(int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                       IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    iec_mc(GetFbVar(AXIS, .AXISNO.value), 1, rise, ex, GetFbVar(DISTANCE), GetFbVar(VELOCITY),
           GetFbVar(ACCELERATION), GetFbVar(DECELERATION), GetFbVar(JERK),
           &GetFbVar(ID), &GetFbVar(ST), &GetFbVar(DONE), &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &nil, &GetFbVar(COMMANDABORTED), &GetFbVar(ACTIVE));
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK

FUNCTION_BLOCK MC_MoveVelocity
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; Velocity : REAL; Acceleration : REAL; Deceleration : REAL; Jerk : REAL; Direction : INT; END_VAR
  VAR_OUTPUT InVelocity : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_mc(int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                       IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    IEC_REAL vel = GetFbVar(DIRECTION) < 0 ? -GetFbVar(VELOCITY) : GetFbVar(VELOCITY);
    iec_mc(GetFbVar(AXIS, .AXISNO.value), 2, rise, ex, 0, vel,
           GetFbVar(ACCELERATION), GetFbVar(DECELERATION), GetFbVar(JERK),
           &GetFbVar(ID), &GetFbVar(ST), &nil, &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &GetFbVar(INVELOCITY), &GetFbVar(COMMANDABORTED), &GetFbVar(ACTIVE));
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK

FUNCTION_BLOCK MC_Halt
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; Deceleration : REAL; Jerk : REAL; END_VAR
  VAR_OUTPUT Done : BOOL; Busy : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_mc(int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                       IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    iec_mc(GetFbVar(AXIS, .AXISNO.value), 3, rise, ex, 0, 0, 0, 0, 0,
           &GetFbVar(ID), &GetFbVar(ST), &GetFbVar(DONE), &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &nil, &GetFbVar(COMMANDABORTED), &nil);
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK

(* MC_Reset (P1 § 3.2): ErrorStop -> Standstill through mc-axis.N.reset (also drives the drive's fault reset);
   Done when mc_axis leaves ErrorStop. Outside ErrorStop mc_axis answers nothing: no Done, Busy drops with Execute. *)
FUNCTION_BLOCK MC_Reset
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; END_VAR
  VAR_OUTPUT Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_mc(int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                       IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    iec_mc(GetFbVar(AXIS, .AXISNO.value), 5, rise, ex, 0, 0, 0, 0, 0,
           &GetFbVar(ID), &GetFbVar(ST), &GetFbVar(DONE), &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &nil, &nil, &nil);
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK

(* MC_SetPosition (P1 § 3.4): the axis's actual position becomes Position (Relative: actual + Position), no motion.
   The shift lives in the AXIS (mc-axis.N.set-position: its offset, like a homing delta; the drive is not touched,
   pos-cmd stays continuous), so actual-position, soft limits and MC_MoveAbsolute of every reader share it.
   Standstill only: elsewhere Error, ErrorID 2 (not allowed in this state). Done/Error by the command number like
   every MC_*. ExecutionMode is not an input (mcImmediately only). *)
FUNCTION_BLOCK MC_SetPosition
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; Position : REAL; Relative : BOOL; END_VAR
  VAR_OUTPUT Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_mc(int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                       IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    iec_mc(GetFbVar(AXIS, .AXISNO.value), 4, rise, ex, GetFbVar(POSITION), GetFbVar(RELATIVE) ? 1 : 0, 0, 0, 0,
           &GetFbVar(ID), &GetFbVar(ST), &GetFbVar(DONE), &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &nil, &nil, &nil);
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK

(* MC_Power (P1 § 3.1, Enable FB): the safety chain permits, the program enables — the component's
   axN-power = Enable while the program runs; the configuration ANDs it with its own power permit into mc-axis.N.power.
   Status = the axis left Disabled (mc-axis.N.state, AxisState: 0 Disabled; ErrorStop keeps the stage on until
   MC_Reset/MC_Power off). Valid follows Enable; Error/ErrorID 1 only for an axis this program does not drive.
   EnablePositive/EnableNegative are not inputs: mc_axis has no per-direction enable (soft limits only). *)
FUNCTION_BLOCK MC_Power
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Enable : BOOL; END_VAR
  VAR_OUTPUT Status : BOOL; Valid : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  {{
    extern int iec_axis_power(int, IEC_BOOL, IEC_BOOL *);
    IEC_BOOL en = GetFbVar(ENABLE), on = 0;
    int ok = iec_axis_power(GetFbVar(AXIS, .AXISNO.value), en, &on);
    SetFbVar(STATUS, on); SetFbVar(VALID, en && ok); SetFbVar(ERROR, !ok); SetFbVar(ERRORID, ok ? 0 : 1);
  }}
END_FUNCTION_BLOCK

(* MC_ReadAxisError (P1 § 3.12, Enable FB): AxisErrorID = why the axis is in ErrorStop (mc-axis.N.axis-error-id,
   0 outside it) — a drive fault at standstill is seen with no command running. Valid/Busy follow Enable. *)
FUNCTION_BLOCK MC_ReadAxisError
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Enable : BOOL; END_VAR
  VAR_OUTPUT Valid : BOOL; Busy : BOOL; Error : BOOL; ErrorID : WORD; AxisErrorID : WORD; END_VAR
  {{
    extern int iec_axis_error(int, IEC_WORD *);
    IEC_BOOL en = GetFbVar(ENABLE), ok = 0; IEC_WORD aid = 0;
    if (en) ok = iec_axis_error(GetFbVar(AXIS, .AXISNO.value), &aid);
    SetFbVar(VALID, en && ok); SetFbVar(BUSY, en && ok); SetFbVar(ERROR, en && !ok); SetFbVar(ERRORID, en && !ok ? 1 : 0);
    SetFbVar(AXISERRORID, aid);
  }}
END_FUNCTION_BLOCK
