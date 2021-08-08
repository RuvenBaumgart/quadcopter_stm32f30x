#include <stdlib.h>
#include <string.h>
#include <stdio.h>

typedef struct bufferToPrint_s{
	char* text;
} bufferToPrint_t;

static bufferToPrint_t introBuffer[] = {
	{"This is the BRIghtFlight FlightController - Intro \n"},
	{"You pluged in the FlightController for setting purpose \n"},
	{"Please select the option \n"},
	{"a = Check the all reveiver input pulses \n"},
	{"b = i2c Scanner \n"},
	{"p = Calibrate the ESC \n"},
	{"q = Quit and show Debugvalues \n"},	
};




