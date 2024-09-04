FUNCTION_BLOCK LSim_IT1
VAR_INPUT
//	calcParam : BOOL;          // recalculate with new parameters
    input     : REAL;          // input signal
    tmLag1    : REAL := 5.0;   // time constant T1 [sec]
    gain      : REAL := 1.0;   // gain factor
    cycle     : REAL := 0.1;   // cyclic time [sec]
    maxOut    : REAL := 100.0; // max. output limit
    minOut    : REAL;          // min. output limit
    reset     : BOOL;          // resets all relevant parameters to ‘0’ including the output
END_VAR
VAR_OUTPUT
	error      : BOOL;   // error (tmLag1 <= 0)
    status     : WORD;   // status (if tmLag1 <= 0 then status = 16#8001)
    maxReached : BOOL;   // output has reached max. limit
    minReached : BOOL;   // output has reached min. limit
    output     : REAL;   // output signal (if error = FALSE)
END_VAR
VAR
	statRxn2   : REAL;   // delayed input signal (n-2)
    statRxn1   : REAL;   // delayed input signal (n-1)
    statRyn2   : REAL;   // delayed output signal (n-2)
    statRyn1   : REAL;   // delayed output signal (n-1)
    statRf1    : REAL;
    statRf2    : REAL;
    statRe1    : REAL;
    statRe2    : REAL;
    statAlpha  : REAL;
    sxInit     : BOOL;
    statOutput : REAL;   // output signal
END_VAR

{region BLOCK INFO HEADER}
//=============================================================================
//(company) / (c)Copyright (year)
//-----------------------------------------------------------------------------
//Library:       Library for Controlled System Simulation
//Tested with:   CODESYS Control Win V3
//Engineering:   CODESYS V3.5 SP20 (64 bit)
//Restrictions:  Cyclic interrupt
//Requirements:  memory card needed for S7-1500
//Functionality: Simulation of a IT1-System
//  
//               gain
//  F(p) = ------------------
//         p * (tmLag1*p + 1)
//         
//  Call the simulation in a cyclic interrupt with
//  cyclic interrupt time at least = tmLag1/10!
//-----------------------------------------------------------------------------
//Change log table:
//Version  Date       Expert in charge                Changes applied
//02.00.00 03.02.2016 Siemens Industry Online Support Release (STEP 7 V13 SP1)
//02.00.01 02.03.2017 Siemens Industry Online Support Release (STEP 7 V14)
//03.00.00 14.06.2024 controltheplant                 Rewitten for Codesys V3.5	
//=============================================================================
{endregion}

// check parameter
IF tmLag1 <= 0 THEN
    error      := TRUE;
    status     := 16#8001; // Parameter Error
    statRxn1   := 0.0;
    statRyn1   := 0.0;
    statRxn2   := 0.0;
    statRyn2   := 0.0;
    statOutput := 0;
    output     := 0;
    RETURN;
ELSE
    error  := FALSE;
    status := 16#0000;     // no error
END_IF;

// reset Memory
IF reset OR NOT sxInit THEN
    statRxn1   := 0.0;
    statRyn1   := 0.0;
    statRxn2   := 0.0;
    statRyn2   := 0.0;
    statOutput :=0;
	sxInit     := TRUE;
END_IF;

// possible improve: calculate exp only if parameters (tmLag or gain) was changed
// calculates the z-parameter for the difference equatation of the IT1-System
statAlpha := EXP(-cycle/tmLag1);
statRe1   := -1 - statAlpha;
statRe2   := statAlpha;
statRf1   := gain*(cycle +(statAlpha - 1)*tmLag1);
statRf2   := gain*(-statAlpha*cycle+(1-statAlpha)*tmLag1);

// save delayed output-signals
statRyn2 := statRyn1;
statRyn1 := statOutput;

// difference equatation for IT1-System
statOutput := statRxn1* statRf1 + statRxn2*statRf2 - statRyn1*statRe1 - statRyn2*statRe2;

// save delayed input-signals
statRxn2 := statRxn1;
statRxn1 := input;

// max limit of acutating variable
IF statOutput > maxOut THEN
    statOutput := maxOut;
    maxReached := TRUE;
ELSE
    maxReached := FALSE;
END_IF;

// min limit of acutating variable
IF statOutput < minOut THEN
    statOutput := minOut;
    minReached := TRUE;
ELSE
    minReached := FALSE;
END_IF;

output := statOutput;