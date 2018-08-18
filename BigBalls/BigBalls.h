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

#define PHOTO_TRANISTOR_PIN 15   // photo transitor PT204-6B. With pullup, read low when bright light present (daylight).

#define PIN_OFF0 16  // wired to U2 pin 6 on PCB  -- main power for Teensy and first LED bank -- OFF high will shut off Teensy
#define PIN_OFF1 21 // wired to RC0 middle pin    -- isolated 5V for second LED bank -- shared ground only
#define PIN_OFF2 14 // wired to RC1 middle pin    -- isolated 5V for third LED bank -- shared ground only

#define VS_PIN 14  // A0 - Battery Voltage after digital switch


// TODO: change these as necessary with testing
#define BRIGHTNESS_AT_DAYTIME  64
#define BRIGHTNESS_AT_NIGHT  64

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
