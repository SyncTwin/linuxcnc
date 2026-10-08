(* SFC qualifier test for matiec: every action body counts its own executions.
   cX     - total body executions of action X
   cXa    - executions while the owning step was active (step.X = TRUE)
   Chart:  S0 --go--> (S1 || S3) ; S1 --nxt--> S4 --nxt--> S2 ; (S2,S3) --back--> S0
   S1: A_P(P) A_P1(P1) A_P0(P0) A_N(N) A_S(S)
   S4: no actions                -- shows that A_S stays set after S1 is left
   S2: A_S(R)  A_N3(R)           -- R on an S action and on an N action of the parallel step S3
   S3: A_N3(N) *)
PROGRAM pulse
  VAR_INPUT
    go   : BOOL := FALSE;
    nxt  : BOOL := FALSE;
    back : BOOL := FALSE;
  END_VAR
  VAR
    cP   : DINT := 0;  cPa  : DINT := 0;
    cP1  : DINT := 0;  cP1a : DINT := 0;
    cP0  : DINT := 0;  cP0a : DINT := 0;
    cN   : DINT := 0;  cNa  : DINT := 0;
    cS   : DINT := 0;  cSa1 : DINT := 0;  cSa4 : DINT := 0;  cSa2 : DINT := 0;
    cN3  : DINT := 0;  cN3a : DINT := 0;  cN3r : DINT := 0;
  END_VAR

  INITIAL_STEP S0:
  END_STEP

  TRANSITION FROM S0 TO (S1, S3)
    := go;
  END_TRANSITION

  STEP S1:
    A_P(P);
    A_P1(P1);
    A_P0(P0);
    A_N(N);
    A_S(S);
  END_STEP

  TRANSITION FROM S1 TO S4
    := nxt;
  END_TRANSITION

  STEP S4:
  END_STEP

  TRANSITION FROM S4 TO S2
    := nxt;
  END_TRANSITION

  STEP S2:
    A_S(R);
    A_N3(R);
  END_STEP

  STEP S3:
    A_N3(N);
  END_STEP

  TRANSITION FROM (S2, S3) TO S0
    := back;
  END_TRANSITION

  ACTION A_P:
    cP := cP + 1;
    IF S1.X THEN cPa := cPa + 1; END_IF;
  END_ACTION

  ACTION A_P1:
    cP1 := cP1 + 1;
    IF S1.X THEN cP1a := cP1a + 1; END_IF;
  END_ACTION

  ACTION A_P0:
    cP0 := cP0 + 1;
    IF S1.X THEN cP0a := cP0a + 1; END_IF;
  END_ACTION

  ACTION A_N:
    cN := cN + 1;
    IF S1.X THEN cNa := cNa + 1; END_IF;
  END_ACTION

  ACTION A_S:
    cS := cS + 1;
    IF S1.X THEN cSa1 := cSa1 + 1; END_IF;
    IF S4.X THEN cSa4 := cSa4 + 1; END_IF;
    IF S2.X THEN cSa2 := cSa2 + 1; END_IF;
  END_ACTION

  ACTION A_N3:
    cN3 := cN3 + 1;
    IF S3.X THEN cN3a := cN3a + 1; END_IF;
    IF S2.X THEN cN3r := cN3r + 1; END_IF;
  END_ACTION
END_PROGRAM

CONFIGURATION config
  RESOURCE res ON PLC
    TASK main(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM inst WITH main : pulse;
  END_RESOURCE
END_CONFIGURATION
