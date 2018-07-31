//
//  BigBalls.hpp
//  Index
//
//  Created by Corbin Dunn on 7/20/18.
//

#ifndef BigBalls_hpp
#define BigBalls_hpp

// A section or group of LEDs
#define NUMBER_GROUPS 24
#define NUMBER_LEDS_PER_GROUP (8+6+8+6) // The pattern Mayra told me
#define NUM_LEDS (NUMBER_LEDS_PER_GROUP*NUMBER_GROUPS)

#define LED_PIN 11 // corbin, using 11 for testing
#define BRIGHTNESS  64
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB

#define DEBUG 1


///////////

#include "LEDPatternType.h" // Defines CD_ENUM


typedef CD_ENUM(int16_t, CDBallPoint)  {
    CDBallPointUp,
    CDBallPointNorth,
    CDBallPointEast,
    CDBallPointSouth,
    CDBallPointWest,
    CDBallPointDown
};



#endif /* BigBalls_hpp */
