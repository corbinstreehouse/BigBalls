//
//  BigBalls.h
//  Index
//
//  Created by Corbin Dunn on 7/20/18.
//

#ifndef BigBalls_hpp
#define BigBalls_hpp

#include "LEDPatterns.h"
#include "Adafruit_BNO055.h"

// A section or group of LEDs
#define NUMBER_GROUPS 24
#define NUMBER_LEDS_PER_GROUP (8+6+8+6) // The pattern Mayra told me
#define NUM_LEDS (NUMBER_LEDS_PER_GROUP*NUMBER_GROUPS)

#define LED_PIN 15 // corbin, using 11 for testing

// Based on Dylan's breakpoint diagram
#define GPS_RX_PIN 7
#define GPS_TX_PIN 8
#define GPS_SERIAL_BAUD 9600

#define BRIGHTNESS  64
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB

#define DEBUG 1

extern LEDPatterns g_patterns;
extern Adafruit_BNO055 g_bno;

extern CRGB *g_LEDs;
extern CRGB *g_LEDsPastEnd; // One past the end of the LED array; so I can wrap to the start when re-mapping

// Ball specific things implemented in each file
extern void initializeBall();

extern void doDirectionalPointWithOrientation(float targetDirectionInDegrees);


#endif /* BigBalls_hpp */
