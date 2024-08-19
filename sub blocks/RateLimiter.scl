FUNCTION_BLOCK RateLimiter
VAR_INPUT
	in     : REAL; 
	rate   : REAL; // maximum rate of change, [1/sec]
	bypass : BOOL := FALSE; 
END_VAR
VAR_OUTPUT
	out          : REAL;
	isIncLimited : BOOL := FALSE; // rate limited when increasing
	isDecLimited : BOOL := FALSE; // rate limited when decreasing
	error        : BOOL;
	errorID      : WORD;
END_VAR
VAR
	slActTime   : LWORD;
	srCycleTime : REAL; // PLC cycle time, [sec]
    trOut       : REAL := 0.0;	
	txIsIncLimited, txIsDecLimited : BOOL; // incdeasing or decreasing is limited
	// дальше статик
	srOld       : REAL := 0.0;
	sxInit      : BOOL := FALSE;	
	slOldTime   : LWORD;
END_VAR
VAR_STAT
	 
END_VAR

VAR CONSTANT
    MICROSECONDS_IN_SEC : REAL := 1.0E-6;
	
    STATUS_NO_ERROR     : WORD := 0;
    ERR_BAD_RATE        : WORD := 16#8203;
END_VAR

error   := FALSE;
errorID := STATUS_NO_ERROR;
{region validation}
IF rate < 0 THEN
	error := TRUE;
	errorID := ERR_BAD_RATE;
	RETURN;
END_IF
{endregion}

IF (NOT sxInit) OR bypass OR (rate = 0.0) THEN  // we always must have correct data from previous cycle 
	sxInit         := TRUE;
	srOld          := in;
	slOldTime      := T_PLC_US();
	trOut          := in;	
	txIsIncLimited := txIsDecLimited := FALSE;
ELSE
{region GET CYCLE TIME}	
    slActTime   := T_PLC_US();	
    srCycleTime := LWORD_TO_REAL(slActTime - slOldTime) * MICROSECONDS_IN_SEC;	
	slOldTime   := slActTime;
{endregion}
	 
	trOut := srOld;	
	IF in < trOut THEN
		trOut := trOut - rate * srCycleTime;
		trOut := MAX(trOut, in);
		srOld := trOut;		
	ELSIF in > trOut THEN
		trOut := trOut + rate * srCycleTime;
		trOut := MIN(trOut, in);
		srOld := trOut;			
	END_IF
	txIsIncLimited := in > trOut;
    txIsDecLimited := in < trOut;
END_IF

out := trOut;
isIncLimited := txIsIncLimited;
isDecLimited := txIsDecLimited;