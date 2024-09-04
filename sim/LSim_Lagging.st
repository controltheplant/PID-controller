FUNCTION_BLOCK LSim_Lagging
VAR_INPUT
	input  : REAL;
END_VAR
VAR_OUTPUT
	output : REAL;
END_VAR
VAR_IN_OUT 
    delayCycles : UINT;
END_VAR
VAR
	statDelayStore : ARRAY[0..N] OF REAL;
	statIndexRead  : INT;	
	statIndexWrite : INT;	
END_VAR
VAR CONSTANT
	N: INT := 100;	
END_VAR

{region BLOCK INFO HEADER}
//=============================================================================
//(company) / (c)Copyright (year)
//-----------------------------------------------------------------------------
//Library:       Library for Controlled System Simulation
//Tested with:   S7-1500 FW V2.0 / S7-1200 FW V4.2
//Engineering:   STEP 7 V14 Upd2 (TIA Portal)
//Restrictions:  Cyclic interrupt
//Requirements:  memory card needed for S7-1500
//Functionality: Simulation of an Tt-System (Lagging)
//
//  F(p) = EXP(-p * delayCycles * cyclic interrupt time)
//          
//  Call the simulation in a cyclic interrupt with
//  cyclic interrupt time at least = tmLag1/10!
//-----------------------------------------------------------------------------
//Change log table:
//Version  Date       Expert in charge                Changes applied
//02.00.00 03.02.2016 Siemens Industry Online Support Release (STEP 7 V13 SP1)
//02.00.01 02.03.2017 Siemens Industry Online Support Optimize program code
//02.00.02 02.03.2017 Siemens Industry Online Support Release (STEP 7 V14)
//03.00.00 07.08.2024 controltheplant                 Release (Codesys V3.5	SP20)
//=============================================================================
{endregion}
	 
IF delayCycles > N THEN
    delayCycles := INT_TO_UINT(N);
ELSE
    delayCycles := delayCycles;
END_IF;

IF statIndexWrite < delayCycles THEN
    statDelayStore[statIndexWrite] := input;         // write input at delayStore indexWrite
    statIndexRead  := statIndexWrite + 1;
    output         := statDelayStore[statIndexRead]; // read output from delayStore indexWrite + 1
    statIndexWrite := statIndexWrite + 1;            // increment delayStore indexWrite
 ELSE // maximum reached
    statDelayStore[statIndexWrite] := input;         // write input at delayStore maximum
    statIndexRead  := 0;                             // set indexRead to beginning
    output         := statDelayStore[statIndexRead]; // read output from delayStore indexRead
    statIndexWrite := 0;                             // set indexWrite to beginning
 END_IF;	 
	 