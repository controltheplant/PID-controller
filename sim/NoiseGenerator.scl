FUNCTION NoiseGenerator : real
VAR_INPUT
	clearValue     : REAL;         // input value wihout noise
	noiseAmplitude : REAL := 0.5;  // relative noise amplitude, [%]
	rangeHi        : REAL := 10;   // actual value upper measurement limit, [eng. unit]
	rangeLo        : REAL := 0;    // actual value lower measurement limit, [eng. unit]
END_VAR
VAR CONSTANT
	PERCENT        : REAL := 0.01;
END_VAR

NoiseGenerator := noiseAmplitude * (OSCAT_BASIC.RDM(last := NoiseGenerator) - 0.5) * PERCENT * (rangeHi - rangeLo) + clearValue;