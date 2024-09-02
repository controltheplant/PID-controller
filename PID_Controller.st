FUNCTION_BLOCK PID_Controller
VAR_INPUT
    setpoint, actualValue : REAL;  // [eng. units] 
    actuatorPos   : REAL := 0.0;   // actual position of actuator (0..100%). Set 0.0 if not used
    offset        : REAL := 0.0;   // disturbance conpensation or precontrol value, [0..100%] 
    trackingValue : REAL;          // setpoint applied in tracking mode, [0..100%]
    track         : BOOL := FALSE; // enable tracking mode
    manual        : BOOL := TRUE;  // enable manual mode
    hold          : BOOL := FALSE; // "freeze" controller's output
    reset         : BOOL := FALSE; // reset output to predefined value and reset errors     
    rangeHi       : REAL := 10.0;  // actual value upper measurement limit, [eng. units]
    rangeLo       : REAL := 0.0;   // actual value lower measurement limit, [eng. units]
END_VAR
VAR_OUTPUT
    out          : REAL;           // controller's output (CO), [0..100%]
    limitsActive : BOOL;           // 1=upper or lower limit of output reached or rate of change of controller's output exceeded (if specified) 
    error        : BOOL  := FALSE;
    errorID      : DWORD := 0;
END_VAR
VAR_IN_OUT
    manualValue  : REAL;           // setpoint for manual mode; in auto mode it follows the controller's output
END_VAR
VAR 
    srCycleTime   : REAL;          // PLC cycle time, [sec]  
    qxIsSaturated : BOOL := FALSE; // controller saturation active
    ixAuto        : BOOL := FALSE; // 1=auto mode
    ixAutoOld     : BOOL := FALSE; // mode in prev cycle
    
    trSetpointScaled, trActualValScaled : REAL; // scaled from eng. units to range 0..100%
    tlCurrentTime                       : LWORD;	
    trSetpointPostRateLimiter           : REAL := 0;   
	trOut                               : REAL := 0; // temporary calculation of `srOut + srDelta`
END_VAR
VAR_STAT
    settings                                   : typePIDSettings;   // structure containing advanced PID settings
    instSetpointFilter, instActualValFilter    : LSim_PT1;
    instSetpointRateLimiter, instActuatorModel : RateLimiter;   
    instOutAverage                             : MovingAverage;
	instFindSteadyState                        : FindSteadyState;
	
    sxInit        : BOOL := FALSE;  // first call
    slOldTime     : LWORD;          // prev cycle system time, [usec]
    srOut         : REAL := 0;      // controller output             
    srError       : REAL;           // control deviation signal
    srErrorOld    : REAL;           // control deviation in prev cycle   
    srDelta       : REAL := 0.0;    // increment for controller output
    srDeltaP      : REAL;           // increment for P-part
    srDeltaI      : REAL;           // increment for I-part
    srDeltaD      : REAL;           // increment for D-part
    srDeltaOffset : REAL;           // adding  offset in incremental form    
    srOffsetOld   : REAL := 0;      // previous offset value
    srDerivative  : REAL;           // derivative part of PID 
    srBackCalc    : REAL;           // value of actuator saturation. Used for back-calculation anti-windup algorithm    
	srPreInt      : REAL;           // value before integral term
	srPreIntOld   : REAL;           // value before integral term on previous plc cycle
    d             : REAL;           // factor for DT1 parameter calculation
    sxSuppressP_Action             : BOOL := FALSE; // 1=suppress P-action 
    qxIsIncLimited, qxIsDecLimited : BOOL;          // 1=increment/decrement of controller's output limited
	qxIsInSteadyState              : BOOL;          // determines whether the process is in steady state
	qxIsInSteadyStateOld           : BOOL;          // steady state in previous cycle
	
	trErrorPreDeadband : REAL;
END_VAR

VAR CONSTANT
    MICROSECONDS_IN_SEC : REAL := 1.0E-6;
	EPS                 : REAL := 1.0E-44;   // smallest real value
	
    STATUS_NO_ERRORS    : WORD := 0;        	
    ERR_BAD_LIMITS      : WORD := 16#8201;   // bad LL, LH
    ERR_CANT_TRACK_SETP : WORD := 16#8202;   // 'TRACK_SETPOINT' method selected, but setpoint ramp is zero    
    ERR_BAD_RATE        : WORD := 16#8203;   // rate of change is negative
    ERR_NO_CYCLE        : WORD := 16#8001;   // PLC cycle time is negative or zero
    ERR_BACK_CALC_T     : WORD := 16#8205;   // bad back calculation factor     
    ERR_BAD_ANTIWINDUP  : WORD := 16#8207;   // error in anti-windup settings    
	ERR_BAD_STAB_TIME   : WORD := 16#8209;   // stabilization time should be positive
	ERR_BAD_DEV_TOL     : WORD := 16#820A;   // deviation tolerance must be in range from 0 to 50%
    ERR_BAD_PID_COEFFS  : WORD := 16#820B;   // wrong PID parameters
    
//     MODE_DEACTIVATE : INT := 0;   // controller inactive, out = 0
//     MODE_AUTOMATIC  : INT := 3;   // automatic control
//     MODE_MANUAL     : INT := 4;   // direct setting of controller output
//     MODE_HOLD       : INT := 5;   // hold controller output
//     MODE_TRACK      : INT := 6;   // output follows the tracking value     
END_VAR

{region BLOCK INFO HEADER} 
//=================================================================================== 
// controltheplant / (C) Copyright (2024) 
//----------------------------------------------------------------------------------- 
// Title:            PID_Controller 
// Comment/Function: PID controller with deadband, filters, rate limiters and more...
// Library/Family:   
// Author:           [controltheplant](https://github.com/controltheplant/PID-controller) 
// Target System:    CODESYS Control Win V3.5.20 
// Engineering:      CODESYS V3.5 SP20 (64 bit) 
// Restrictions:     Execute in cyclic interrupt 
// Requirements:     
//----------------------------------------------------------------------------------- 
// Change log table: 
// Version     | Date       | Expert in charge | Changes applied 
//-------------|------------|------------------|------------------------------------- 
//   1.0.0     | 2024-08    | controltheplant  | First released version 
//   1.1.0     | 2024-09    | controltheplant  |
//               - back calculation function is converted to the form used in Matlab;  
//               - further adaptation of various functions to incremental form.
//=================================================================================== 
{endregion}

{region DESCRIPTION}
(* 
PID algorithm:
    Velocity-based aka incremental algorithm is used. Controller output (CO) is calculated as previous CO + delta.
    Delta is change of CO per one cycle. Ideal- aka ISA-form of PID-algorithm is used. Use 'settings' structure to tune the controller.
    Use OutRampRate only for safety of Controllable Object!      

Anti-Windup:
    CLAMPING: integrator stops integration when output reaches the limits
    BACK_CALC_RAMP: back calculation algorithm based on model of actuator, whitch is a rate limiter + saturation
    BACK_CALC_REAL: back calculation algorithm based on measured position of actuator. Signal of actuator's position required

Bumpless transfer:
    Auto->Manual : manual setpoint tracks the controller output
    Manual->Auto : TRACK SETPOINT: setpoint tracks the actual value; setpoint returns back to the value on the input along the ramp. 
                   'SetpointRampRate' in 'settings' must be set!
                   SUPPRESS P-ACTION: proportional part is suppressed after switching to auto returned back when the P-part becomes close to zero   
  
 Scaling                                                                 offset
    |>        ________     ______________           __________     _____    |     ______________     _____        _______
 SP |  >     |        |   |              |         |          |   |     |   v    |              |   |   _ | CO   |       |   PV
--->| S  >-->| Filter |-->| Rate Limiter |-->(-)-->| Deadband |-->| PID |--(+)-->| Rate Limiter |-->| _/  |----->| Plant |--+-->
    |  >     |________|   |______________|    ^    |__________|   |_____|        |______________|   |_____|      |_______|  |
    |>                                        |                                                                             |
                                              |            ________               <|                                        |
                                              |   PV_FLT  |        |            <  |                                    PV  |
                                              +-----------| Filter |<---------<  S |<---------------------------------------+
                                                          |________|            <  |
                                                                                  <|
*)       
{endregion}

{region INITIALIZATION}
IF NOT sxInit THEN
    error         := FALSE;
    errorID       := STATUS_NO_ERRORS;
    sxInit        := TRUE;
    slOldTime     := T_PLC_US();
    srError       := 0;
    srOffsetOld   := offset;
    srPreInt      := 0;
    srDerivative  := 0;       
    srOut         := manualValue;
    RETURN;
END_IF
{endregion}

{region GET CYCLE TIME}
tlCurrentTime := T_PLC_US();
srCycleTime   := LWORD_TO_REAL(tlCurrentTime - slOldTime) * MICROSECONDS_IN_SEC;
slOldTime     := tlCurrentTime;
IF srCycleTime <= EPS THEN
    error   := TRUE;
    errorID := ERR_NO_CYCLE;
    RETURN;
END_IF
{endregion}

{region VALUE SCALING}
// check for incorrect limits
IF     (rangeLo >= rangeHi)
    OR (settings.irOutLimitL >= settings.irOutLimitH) 
THEN 
    error   := TRUE;
    errorID := ERR_BAD_LIMITS;
    RETURN;      
END_IF
// Scaling of setpoint and actual values must be the same, as well as the range of output
trActualValScaled := SCALE_R(X := actualValue,
                          I_LO := rangeLo, 
                          I_HI := rangeHi, 
                          O_LO := 0, 
                          O_HI := 100.0);
trSetpointScaled  := SCALE_R(X := setpoint,
                          I_LO := rangeLo, 
                          I_HI := rangeHi, 
                          O_LO := 0, 
                          O_HI := 100.0);                               
{endregion}

{region MODE SELECTION}
    ixAutoOld := ixAuto;
    ixAuto    := NOT (manual OR hold OR reset OR track);   
{endregion}

{region SETPOINT TRACKING}
// when transfer to Manual, setpoint is tracking the actual value
// when the mode returns back to Auto, setpoint goes to the value at the block input along the ramp
IF (NOT ixAuto) 
    AND settings.iwBumplessTransferMethod = enumBumplessTransferMethods.TRACK_SETPOINT
THEN
    trSetpointScaled := trActualValScaled;      
END_IF
IF     (settings.iwBumplessTransferMethod = enumBumplessTransferMethods.TRACK_SETPOINT)
   AND (settings.irSetpointRampRate <= EPS)
THEN
    error   := TRUE;
    errorID := ERR_CANT_TRACK_SETP;     
END_IF
{endregion}

{region FILTERING AND RATE LIMIT}
// applicable in cascade control, in other cases set T:= 0.0
instSetpointFilter(input  := trSetpointScaled,
                   T      := settings.itSetpointFilterTime, 
                   cycle  := srCycleTime, 
                   bypass := NOT ixAuto);  // in non-automatic modes setpoint passes directly, without filtering
IF instSetpointFilter.error THEN
    error   := TRUE;
    errorID := instSetpointFilter.errorID;
END_IF                      
// set rate to zero for tuning, in other cases set rate to non-zero value
// Setpoint rate limiter is necessary when 'TRACK_SETPONT' selected as bumpless transfer method in 'settings'   
instSetpointRateLimiter(in     := instSetpointFilter.out,  
                        rate   := settings.irSetpointRampRate,
                        bypass := NOT ixAuto,
                        out    => trSetpointPostRateLimiter);
IF instSetpointRateLimiter.error THEN
    error   := TRUE;
    errorID := instSetpointRateLimiter.errorID;
    RETURN;
END_IF;
instActualValFilter(input := trActualValScaled, 
                    T     := settings.itActValueFilterTime, 
                    cycle := srCycleTime);
{endregion}

{region CONTROL DEVIATION CALCULATION AND DEADBAND}
srErrorOld := srError;
IF settings.irDeadband < 0.0 THEN
    settings.irDeadband := ABS(settings.irDeadband);
END_IF
IF (settings.irDeviationTolerance <= EPS) OR (settings.irDeviationTolerance > 50.0) THEN
    error   := TRUE;
    errorID := ERR_BAD_DEV_TOL;
END_IF 
IF settings.irStabilizationTime < 0 THEN
    error   := TRUE;
    errorID := ERR_BAD_STAB_TIME;
END_IF
IF settings.ixInvert THEN       
    srError := instActualValFilter.out - trSetpointPostRateLimiter;                            
ELSE    
    srError := trSetpointPostRateLimiter - instActualValFilter.out;                            
END_IF
qxIsInSteadyStateOld := qxIsInSteadyState;
instFindSteadyState(value             := srError, 
                    tolerance         := settings.irDeadband, 
                    stabilizationTime := settings.irStabilizationTime, 
                    cycleTime         := srCycleTime,
                    isInSteadyState   => qxIsInSteadyState,
                    error             => error,
                    errorID           => errorID);    
trErrorPreDeadband := srError;   // just to monitor the error before the dead zone
srError := DeadBand(X        := srError, 
                    deadZone := settings.irDeadband * BOOL_TO_REAL(qxIsInSteadyState));  // dead zone is applied only in steady state 
{endregion}

{region CONTROLLER} // velocity (incremental) PID algorithm
{region CHECK PID PARAMETERS}   
IF    (settings.irKP < 0)          // bad PID parameters
   OR (settings.irTI < 0)
   OR (settings.irTD < 0)
   OR (settings.irTdF < 0)
THEN
    error   := TRUE;
    errorID := ERR_BAD_PID_COEFFS;
    RETURN;     
END_IF
{endregion}
        
IF ixAuto THEN 
    srDeltaP := srError - srErrorOld;   
    {region SUPPRESS P-ACTION}  
        IF     (settings.iwBumplessTransferMethod = enumBumplessTransferMethods.SUPPRESS_P_ACTION)
           AND (NOT ixAutoOld)                                      // trigger for switching from manual to auto
           AND (ABS(srError) > settings.irDeviationTolerance)   
        THEN
            sxSuppressP_Action := TRUE;                                                       
        ELSIF ABS(srDeltaP)*settings.irKP <= settings.irDeviationTolerance THEN   // switch on P-action back when proportional part is close to zero
            sxSuppressP_Action := FALSE;
        END_IF
    {endregion} 
    IF settings.irTI <= EPS THEN // check for validity of integral time
        srDeltaI := 0;
    ELSE    
        srPreIntOld := srPreInt;
        srPreInt    := srError / settings.irTI - srBackCalc;        
        srDeltaI    := srCycleTime * 0.5 * (srPreInt + srPreIntOld); // Trapezoid method
    END_IF
// get derivative component  
    IF     (settings.irTdF <= EPS) 
        OR (settings.irTd  <= EPS) 
    THEN // check for validity of filter time in derivative channel
        srDeltaD     := 0;
        srDerivative := 0;
    ELSE
        d            := EXP(-srCycleTime/settings.irTdF);
        srDeltaD     := (srError - srErrorOld) * settings.irTD / settings.irTdF + srDerivative*(d-1); // DT1, real drivative
        srDerivative := srDerivative + srDeltaD;             // derivative component
    END_IF  
    // offset component
    srDeltaOffset := offset - srOffsetOld;                   
    srOffsetOld   := offset;   
    
    srDelta := (srDeltaP + srDeltaI + srDeltaD) * settings.irKP + srDeltaOffset; // Ideal (ISA) form,   preliminary calculation
    trOut   := srOut + srDelta;
    IF sxSuppressP_Action THEN
        srDelta := (srDeltaI + srDeltaD) * settings.irKP + srDeltaOffset;
    END_IF  
    IF settings.irOutRampRate < 0.0 THEN
        error   := TRUE;
        errorID := ERR_BAD_RATE;
        RETURN;
    END_IF      
    {region ANTI WINDUP}        
    CASE settings.iwAntiWindupMethod OF 
    enumAntiWindupMethods.CLAMPING :
         
        srBackCalc := 0;              
        // integration is limited to prevent saturation
        IF trOut > settings.irOutLimitH THEN
            srDelta := settings.irOutLimitH - srOut;
        END_IF
        IF trOut < settings.irOutLimitL THEN
            srDelta := settings.irOutLimitL - srOut;
        END_IF      
     
    enumAntiWindupMethods.BACK_CALC_MODEL :
     
        IF settings.irBackCalcFactor <= EPS THEN
            error   := TRUE;
            errorID := ERR_BACK_CALC_T;   
        END_IF
        instActuatorModel(in     := trOut,
                          rate   := settings.irOutRampRate);    
        srBackCalc  := (trOut - LIMIT(settings.irOutLimitL, instActuatorModel.out , settings.irOutLimitH)) / settings.irBackCalcFactor;                                          
        srPreInt    := srError / settings.irTI - srBackCalc;     
        srDeltaI    := srCycleTime * 0.5 * (srPreInt + srPreIntOld); // Trapezoid method
        srDelta     := (srDeltaP + srDeltaI + srDeltaD) * settings.irKP + srDeltaOffset;   // recalculation with new `srBackCalc` value      
    
    enumAntiWindupMethods.BACK_CALC_REAL : 
     
        IF settings.irBackCalcFactor <= EPS THEN
            error   := TRUE;
            errorID := ERR_BACK_CALC_T;   
        END_IF    
        srBackCalc  := (trOut - LIMIT(settings.irOutLimitL, actuatorPos, settings.irOutLimitH)) / settings.irBackCalcFactor;		
        srPreInt    := srError / settings.irTI - srBackCalc;     
        srDeltaI    := srCycleTime * 0.5 * (srPreInt + srPreIntOld); // Trapezoid method
        srDelta     := (srDeltaP + srDeltaI + srDeltaD) * settings.irKP + srDeltaOffset;   // recalculation with new `srBackCalc` value
        
    ELSE
        error   := TRUE;
        errorID := ERR_BAD_ANTIWINDUP;
        RETURN;
    END_CASE
    {endregion}     
    
    {region CO MEAN VALUE}
    // this is ensure the actual value is in the middle of the dead zone,
    // and not at the edge of the dead zone 
    instOutAverage(enable      := (settings.irDeadband > 0.0) AND NOT qxIsInSteadyState,
                   x           := trOut, 
                   sizeOfArray := REAL_TO_INT(settings.irStabilizationTime / srCycleTime), 
                   reset       := reset,    
                   error       => error, 
                   errorID     => errorID);                      
    IF qxIsInSteadyState AND (NOT qxIsInSteadyStateOld) THEN
        srDelta := instOutAverage.averageValue - srOut;         // equivalent of `srOut := instOutAverage.averageValue`         
    END_IF;    
    {endregion} 
    
    manualValue := trOut;    // manual value is tracking the CO in Auto mode                   
ELSIF manual THEN
    srDelta := LIMIT(settings.irOutLimitL, manualValue, settings.irOutLimitH) - srOut;     
ELSIF track THEN
    srDelta := LIMIT(settings.irOutLimitL, trackingValue, settings.irOutLimitH) - srOut;    
END_IF      
IF (settings.irOutRampRate > 0.0) THEN
    // rate limit for output
    srDelta := LIMIT(-settings.irOutRampRate * srCycleTime, 
                      srDelta,
                      settings.irOutRampRate * srCycleTime);  
END_IF
// controller output    
srOut := srOut + srDelta;   
{endregion}  

{region RESET}
IF reset THEN   
    srOut   := LIMIT(settings.irOutLimitL ,settings.irResetValue, settings.irOutLimitH); 
    error   := FALSE;
    errorID := STATUS_NO_ERRORS;
END_IF
{endregion}

{region CHECK LIMITS AND RATE}
qxIsSaturated  := (srOut >= settings.irOutLimitH) OR (srOut <= settings.irOutLimitL);
qxIsIncLimited := (srDelta >= settings.irOutRampRate * srCycleTime   
                  AND settings.irOutRampRate > 0);
qxIsDecLimited := (srDelta <= -settings.irOutRampRate * srCycleTime 
                  AND settings.irOutRampRate > 0);  
{endregion}

{region OUT}   
out          := LIMIT(settings.irOutLimitL, srOut, settings.irOutLimitH);
limitsActive := qxIsIncLimited OR qxIsDecLimited OR qxIsSaturated;
{endregion}