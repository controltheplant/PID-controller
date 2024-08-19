FUNCTION_BLOCK FindSteadyState
VAR_INPUT
	value             : REAL;          // input value, [eng. units or %]
	tolerance         : REAL := 1.0;   // ripple amplitude of input value in steady state, [eng. units or %]
	stabilizationTime : REAL := 0.5;   // time during which the input value should lie whithin tolarance to determine steady state, [s]
	cycleTime         : REAL := 0.1;   // plc cycle time, [sec]
END_VAR
VAR_OUTPUT
	isInSteadyState   : BOOL := FALSE; // 1 = in steade state, 0 = in transition process 
	error             : BOOL := FALSE;
	errorID           : DWORD; 
END_VAR
VAR
	instMovingAverage      : MovingAverage;
    instStabilizationTimer : TON;  // timer to count the stabilization time		
END_VAR
VAR CONSTANT
	MILLISECONDS_IN_SEC : REAL := 1000;
	
	NO_ERROR            : WORD := 16#0;
	ERR_BAD_TOLERANCE   : WORD := 16#820A;
	ERR_BAD_STAB_TIME   : WORD := 16#8209;
END_VAR

{region Description header}
//================================================================================== 
// controltheplant / (C) Copyright (2024) 
//---------------------------------------------------------------------------------- 
// Title: IsInSteadyState 
// Comment/Function: Determines whether the process value is in steady state 
// Library/Family:   PID-Controller
// Author:           controltheplant 
// Target System:    CODESYS Control Win V3  
// Engineering:      CODESYS V3.5 SP20 (64 bit) 
// Restrictions:     no 
// Requirements:     no
//---------------------------------------------------------------------------------- 
// Change log table: 
// Version     | Date       | Expert in charge | Changes applied 
//-------------|------------|------------------|------------------------------------ 
//       1.0.0 | 2024-08    | controltheplant  | First released version 
//================================================================================== 
{endregion}

{region DESCRIPTION}
// Block returns TRUE when the 'value' input stabilizes around zero, i.e. moving average of 
// 'value' doesn't exceed 'tolerance' by modulo during 'stabilizationTime'	
{endregion}

error   := FALSE;
errorID := NO_ERROR;
IF stabilizationTime < 0.0 THEN
	error := TRUE;
	errorID := ERR_BAD_STAB_TIME;
	RETURN;
END_IF
IF tolerance <= 0.0 THEN
	error := TRUE;
	errorID := ERR_BAD_TOLERANCE;
	RETURN;
END_IF
instMovingAverage(enable := TRUE, 
                       x := value, 
             sizeOfArray := REAL_TO_INT(stabilizationTime / cycleTime));
instStabilizationTimer(IN := ABS( (* instMovingAverage.averageValue *) value) <= tolerance, 
                       PT := REAL_TO_TIME(stabilizationTime * MILLISECONDS_IN_SEC), 
                        Q => isInSteadyState);
					   