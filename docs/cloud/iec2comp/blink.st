PROGRAM blink
  VAR
    lamp AT %QX0.0 : BOOL := FALSE;
  END_VAR
  VAR
    n : DINT := 0;
  END_VAR
  n := n + 1;
  lamp := (n MOD 2) = 1;
END_PROGRAM
