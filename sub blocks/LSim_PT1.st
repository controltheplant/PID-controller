FUNCTION_BLOCK LSim_PT1
VAR_INPUT
    input     : REAL;      // input signal
    T         : REAL := 1.0;   // time constant T1 [sec] 
    cycle     : REAL := 0.1;   // cyclic time [sec]
    bypass    : BOOL;      // resets all relevant parameters to ‘0’ including the output
END_VAR
VAR_OUTPUT
	error   : BOOL;   // error (tmLag1 <= 0)
    errorID : WORD;   // status (if tmLag1 <= 0 then status = 168001)    
    out     : REAL;   // output signal (if error = FALSE)
END_VAR
VAR
	statRf1    : REAL;
    statRe1    : REAL;    
    statOutput : REAL;
END_VAR
VAR CONSTANT
	ERR_BAD_FILTER_TIME : WORD := 16#8202;
	STATUS_NO_ERRORS    : WORD := 0;
END_VAR

{region BLOCK INFO HEADER}
 //===============================================================================
 // (company) / (c)Copyright (2024)
 //-------------------------------------------------------------------------------
 // Title:            PT1
 // Comment/Function: First order filter (low-pass)
 // Library/Family:   LSim
 // Author:           controltheplant
 // Tested with:      Codesys Control Win V3
 // Engineering:      Codesys 64 V3.5.19
 // Restrictions:     cyclic interrupt
 // Requirements:     (hardware, technological package, memory needed, etc.)
 //-------------------------------------------------------------------------------
 // Change log table:
 // Version  | Date       | Expert in charge       | Changes applied
 //----------|------------|------------------------|------------------------------
 // 01.00.00 | 2023-09-18 |                        | First released version
 //===============================================================================
{endregion}

{region DESCRIPTION}
(*
FB “LSim_PT1” simulates a PT1 element. The PT1 element is a proportional transfer element with first-order order delay.
              gain
  F(s) = ---------------
          tmLag1 * s + 1
*)
{endregion}

IF T < 0 THEN
    error := TRUE;
    errorID := ERR_BAD_FILTER_TIME; // Parameter Error
    statOutput := 0;
    out := 0;
    RETURN;
ELSIF T = 0.0 THEN
	statOutput := input;
	out := statOutput;
	RETURN;
ELSE
    error := FALSE;
    errorID := STATUS_NO_ERRORS; // no error
END_IF;

IF bypass THEN
    statOutput := input;
	RETURN;
END_IF;

// possible improvement: calculate exp only when parameter T changes
// Z-parameters For Laplace PT1-System
statRe1 := - EXP(-cycle/T);
statRf1 := 1 + statRe1;

// Differencial Equation of PT1-System
statOutput := input*statRf1 - statOutput*statRe1;		
	
out := statOutput;