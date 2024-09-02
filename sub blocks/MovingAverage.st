FUNCTION_BLOCK MovingAverage
VAR_INPUT
	enable      : BOOL := FALSE;  // TRUE: Enable function block 
    x           : REAL := 0.0;    // Input value
    sizeOfArray : INT  := 20;     // Number of values to calculate avarage value of [1-20]
    reset       : BOOL := FALSE;  // TRUE: Reset avarage value calculation
END_VAR
VAR_OUTPUT	
    error           : BOOL  := FALSE;          // TRUE: An error occured
    errorID         : DWORD := 16#0000_0000;   // Detailed error description
    averageValue    : REAL  := 0.0;            // Average value
    maxValueReached : BOOL  := FALSE;          // Defined number of values are used for average value calculation (internal array is filled)
END_VAR
VAR
	sxMaxValueReached : BOOL;
    instTrigEn        : R_TRIG;
    srBufferedValues  : ARRAY[0..MAX_VALUE_BREAK_POINT_TABLE] OF REAL := [199(0.0)];
    siArrayCounter    : INT;
    siLimitedSize     : INT;
    siMaxValue        : INT;
    srTotalValue      : REAL;
END_VAR
VAR CONSTANT
    MAX_LENGTH_AVERAGE_VALUE                 : INT   := 200; //20
    NO_ERROR                                 : DWORD := 16#0000_0000;
	MAX_VALUE_BREAK_POINT_TABLE              : INT   := 199;
	LCON_ERROR_AVARAGE_INVALID_CONFIGURATION : DWORD := 16#0105_8000; // Invalid configuration data for AverageValue calculation:  [sizeOfArray < 2],  [sizeOfArray > 20]
END_VAR

instTrigEn(CLK := enable);

IF NOT enable THEN
	averageValue := x;
	RETURN;
END_IF

IF reset OR instTrigEn.Q THEN                           //Reset values on reset command or on rising trigger pulse	
	siLimitedSize     := LIMIT(1, sizeOfArray, MAX_LENGTH_AVERAGE_VALUE);
	error             := FALSE;
	errorID           := NO_ERROR;
	siArrayCounter    := 0;
	sxMaxValueReached := FALSE;
	srTotalValue      := 0.0;
END_IF;

//ERRORS
IF (sizeOfArray > MAX_LENGTH_AVERAGE_VALUE)
	OR (sizeOfArray < 2)
THEN
	error   := TRUE;
	errorID := LCON_ERROR_AVARAGE_INVALID_CONFIGURATION;
END_IF;

IF siArrayCounter > (siLimitedSize - 1) THEN      // Fill Array with Input Value
	siArrayCounter    := 0;                       // Begin at Zero
	sxMaxValueReached := TRUE;
END_IF;

IF sxMaxValueReached THEN
	srTotalValue := srTotalValue - srBufferedValues[siArrayCounter];
END_IF;

srBufferedValues[siArrayCounter] := x;   // Write new Value in ARRAY
siArrayCounter := siArrayCounter + 1;    // Set counter up
IF sxMaxValueReached THEN                // Choose max index
	siMaxValue := siLimitedSize;
ELSE
	siMaxValue := siArrayCounter;
END_IF;

srTotalValue := srTotalValue + srBufferedValues[siArrayCounter - 1];      // (arrayCounter-1)----->present value
averageValue := srTotalValue / siMaxValue;                                // Calc Average

maxValueReached := sxMaxValueReached;
