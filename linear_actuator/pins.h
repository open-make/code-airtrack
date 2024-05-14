#ifndef PINS_MODULE
#define PINS_MODULE

#import <Arduino.h>
#include "definitions.h"
// which pins are used for what
struct PinStruct
{
	CONST_PIN_TYPE Sensor = 2;

    CONST_PIN_TYPE ActuatorPush = 4;
    CONST_PIN_TYPE ActuatorPull = 3;

    // CONST_PIN_TYPE SolenoidLeft = 8;
    // CONST_PIN_TYPE SolenoidRight = 5;
    // switch right and left, don't forget to change sensor pins
    CONST_PIN_TYPE SolenoidLeft = 5;
    CONST_PIN_TYPE SolenoidRight = 8;

    CONST_PIN_TYPE LaneLight = 6;

    CONST_PIN_TYPE PeizoTone = 7;
} Pins;

#endif
