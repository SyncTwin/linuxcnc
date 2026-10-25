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
   ⚠ MATIEC runs the final scan of an N action whose step left (IEC 61131-3 2.6.4.5, Q FALSE; generated C:
   pass 0 of `Actions execution`): the FB is called once more with Execute = step.X = FALSE, which clears
   Done/Error/ErrorID — read them in the step's own action, never from the next step (RP32 Transfer 09.10:
   StopReason took the cleared ErrorID 0, no Hold, 329 retries). An FB not called last tick had Execute FALSE —
   hence LastTick, so a re-entry is a rising edge however the step was left. *)
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

(* MC_ReadActualPosition (PLCopen TC2 Motion Control Part 1 v2.0, 2011, administrative FB; Enable FB):
   Position = mc-axis.N.actual-position (pos-fb + the MC_SetPosition offset, axis frame — the same value
   MC_MoveAbsolute and the soft limits use), read every scan while Enable. Valid/Busy follow Enable;
   Error/ErrorID 1 only for an axis this program does not drive (as MC_ReadAxisError). Position keeps its last
   value when Enable falls. *)
FUNCTION_BLOCK MC_ReadActualPosition
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Enable : BOOL; END_VAR
  VAR_OUTPUT Valid : BOOL; Busy : BOOL; Error : BOOL; ErrorID : WORD; Position : REAL; END_VAR
  {{
    extern int iec_axis_position(int, IEC_REAL *);
    IEC_BOOL en = GetFbVar(ENABLE), ok = 0; IEC_REAL pos = GetFbVar(POSITION);
    if (en) ok = iec_axis_position(GetFbVar(AXIS, .AXISNO.value), &pos);
    SetFbVar(VALID, en && ok); SetFbVar(BUSY, en && ok); SetFbVar(ERROR, en && !ok); SetFbVar(ERRORID, en && !ok ? 1 : 0);
    SetFbVar(POSITION, pos);
  }}
END_FUNCTION_BLOCK

(* MC_ReadActualTorque (PLCopen TC2 Motion Control Part 1 v2.0, 2011, administrative FB; Enable FB):
   Torque = mc-axis.N.torque-fb — the drive's torque actual value, object 0x6077 of CiA 402 as the drive reports it:
   per mille (0.1 %) of the rated torque 0x6076, signed, no scaling on the way (a live drive through lcec, a
   virtual one through the ESC responder of the box, the same signal and unit). Read every scan while Enable;
   Valid/Busy follow Enable; Error/ErrorID 1 only for an axis this program does not drive (as MC_ReadActualPosition).
   Torque keeps its last value when Enable falls. *)
FUNCTION_BLOCK MC_ReadActualTorque
  VAR_IN_OUT Axis : AXIS_REF; END_VAR
  VAR_INPUT Enable : BOOL; END_VAR
  VAR_OUTPUT Valid : BOOL; Busy : BOOL; Error : BOOL; ErrorID : WORD; Torque : REAL; END_VAR
  {{
    extern int iec_axis_torque(int, IEC_REAL *);
    IEC_BOOL en = GetFbVar(ENABLE), ok = 0; IEC_REAL trq = GetFbVar(TORQUE);
    if (en) ok = iec_axis_torque(GetFbVar(AXIS, .AXISNO.value), &trq);
    SetFbVar(VALID, en && ok); SetFbVar(BUSY, en && ok); SetFbVar(ERROR, en && !ok); SetFbVar(ERRORID, en && !ok ? 1 : 0);
    SetFbVar(TORQUE, trq);
  }}
END_FUNCTION_BLOCK

(* MC_GearIn (P1 § 4.5 p. 94): Slave runs at RatioNumerator / RatioDenominator x Master (mc-gear.<Slave>, mc_gear(9)).
   The slave ramps in with Acceleration/Deceleration/Jerk (0 = the slave's maxa / = Acceleration / none), the distance
   lost ramping is not caught up (note 1); InGear when coupled; a new GearIn changes the ratio on the fly (note 2).
   Busy stays while geared (as MC_MoveVelocity after InVelocity) until MC_GearOut, another GearIn or MC_Stop of the
   slave (CommandAborted). From Standstill of the slave only (mc_axis), elsewhere Error 2. Master must be the axis the
   configuration wired to mc-gear.<Slave>.master-pos, else Error 11. MasterValueSource and BufferMode are not inputs:
   the source is the wiring (commanded or actual position), the buffer mode mcAborting. *)
FUNCTION_BLOCK MC_GearIn
  VAR_IN_OUT Master : AXIS_REF; Slave : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; RatioNumerator : INT := 1; RatioDenominator : UINT := 1;
            Acceleration : REAL; Deceleration : REAL; Jerk : REAL; END_VAR
  VAR_OUTPUT InGear : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_gear(int, int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                         IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    iec_gear(GetFbVar(SLAVE, .AXISNO.value), GetFbVar(MASTER, .AXISNO.value), 2, rise, ex,
             GetFbVar(RATIONUMERATOR), GetFbVar(RATIODENOMINATOR), GetFbVar(ACCELERATION), GetFbVar(DECELERATION), GetFbVar(JERK),
             &GetFbVar(ID), &GetFbVar(ST), &nil, &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &GetFbVar(INGEAR),
             &GetFbVar(COMMANDABORTED), &GetFbVar(ACTIVE));
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK

(* MC_GearOut (P1 § 4.6 p. 96): the slave leaves the gear; Done in the period the gear lets go. Deviation: the slave
   decelerates to Standstill at its maxa (mc_axis), not ContinuousMotion at its last velocity. Not geared: Error 2. *)
FUNCTION_BLOCK MC_GearOut
  VAR_IN_OUT Slave : AXIS_REF; END_VAR
  VAR_INPUT Execute : BOOL; END_VAR
  VAR_OUTPUT Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : WORD; END_VAR
  VAR Prev : BOOL; Id : UDINT; St : USINT; LastTick : UDINT; END_VAR
  {{
    extern IEC_UDINT iec_tick;
    extern void iec_gear(int, int, int, int, int, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_REAL, IEC_UDINT *, IEC_USINT *,
                         IEC_BOOL *, IEC_BOOL *, IEC_BOOL *, IEC_WORD *, IEC_BOOL *, IEC_BOOL *, IEC_BOOL *);
    IEC_BOOL ex = GetFbVar(EXECUTE), nil = 0;
    IEC_BOOL rise = ex && !(GetFbVar(PREV) && GetFbVar(LASTTICK) + 1 == iec_tick);
    iec_gear(GetFbVar(SLAVE, .AXISNO.value), -1, 3, rise, ex, 0, 0, 0, 0, 0,
             &GetFbVar(ID), &GetFbVar(ST), &GetFbVar(DONE), &GetFbVar(BUSY), &GetFbVar(ERROR), &GetFbVar(ERRORID), &nil, &nil, &nil);
    SetFbVar(PREV, ex); SetFbVar(LASTTICK, iec_tick);
  }}
END_FUNCTION_BLOCK
