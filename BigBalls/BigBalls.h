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
#define NUMBER_LEDS_PER_GROUP (8*4) // 4 rows of 8 in a square
#define NUM_LEDS (NUMBER_LEDS_PER_GROUP*NUMBER_GROUPS)

#define LED_PIN 13 // ?
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
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

//typedef CD_ENUM(int16_t, CDBallDirection)  {
//    CDBallDirectionNorthWestUp = 0,
//
//    CDWheelCommandNextPattern = CDWheelCommandFirst,
//    CDWheelCommandPriorPattern = 1,
//    CDWheelCommandNextSequence = 2,
//    CDWheelCommandPriorSequence = 3,
//    CDWheelCommandRestartSequence = 4,
//    CDWheelCommandStartCalibrating = 5,
//    CDWheelCommandEndCalibrating = 6,
//    CDWheelCommandCancelCalibrating = 7,
//    CDWheelCommandStartSavingGyroData = 8,
//    CDWheelCommandEndSavingGyroData = 9,
//
//    CDWheelCommandPlay = 10,
//    CDWheelCommandPause = 11,
//
//    CDWheelCommandLast = CDWheelCommandPause,
//    CDWheelCommandCount = CDWheelCommandLast + 1,
//};








#endif /* BigBalls_hpp */
