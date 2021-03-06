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

#define LED_PIN 2 // corbin, using 11 for testing

// Based on Dylan's breakpoint diagram
#define GPS_RX_PIN 7
#define GPS_TX_PIN 8
#define GPS_SERIAL_BAUD 9600

#define PHOTO_TRANISTOR_PIN 15   // photo transitor PT204-6B. With pullup, read low when bright light present (daylight).

// Battery voltage pins and values
static const int VOLTAGE_READ_PINS[3] = { 16 /* battery 0*/, 20 /* battery 1*/, 21 /* battery 2*/ };
static const int VOLTAGE_SHUTOFF_PINS[3] = { 4, 20, 14 /* TODO Update */ };

#define MIN_BATTERY_VOLTAGE  (5*3.3) // VOLTS, for 2-cell lipo


#define REF_VOLTAGE 3.3 // TODO: this could be measured
#define RESISTOR_Z1_VALUE 100.0 // kOhm
#define RESISTOR_Z2_VALUE 15.0 // kOhm   (with 10uF filter cap in parallel)

// TODO: change these as necessary with testing; 0-128
#define BRIGHTNESS_AT_DAYTIME  64
#define BRIGHTNESS_AT_NIGHT  64

#define BRIGHTNESS_WAITING_AT_DAYTIME  32
#define BRIGHTNESS_WAITING_AT_NIGHT  32

// if it is within X meters, start pointing away
#define DISTANCE_TO_FLIP_TO_POINTING_AWAY 10 // in meters
// If it is pushed 300 meters (~1000 feet) away, switch and start pointing in.
#define DISTANCE_TO_FLIP_TO_POINTING_TOWARDS_TOWER 300


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
