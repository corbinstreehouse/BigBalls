//
//  PlasticBall.cpp
//  Index
//
//  Created by Corbin Dunn on 7/30/18 .
//

#include "PlasticBall.h"
#include "BigBalls.h"

// Things specific to the plastic ball

// These are defined in BigBalls.h, but probably should be hidden and only in this location
#if DEBUG
    #define NUMBER_LEDS_PER_GROUP (1) // corbin testing!! so I can use a small strip
#else
    #define NUMBER_LEDS_PER_GROUP (8+6+8+6)
#endif

#define POINT_COLOR CRGB::Green

#define NUMBER_GROUPS_PER_CARDINAL_POINT 4 // four pentagons per cardinal point/direction

#define DEGREE_VARIATION_FOR_CARDINAL_POINT 10 // if we are within <value> degrees from a cardinal point, we will highlight all four pentagons. Otherwise, we higlight two (or maybe one)

static inline CRGB *wrapPointIfNeeded(CRGB *point) {
    int amountPast = g_LEDsPastEnd - point;
    if (amountPast >= 0) {
        return &g_LEDs[amountPast];
    } else {
        return point;
    }
}

static inline void highlightCardinalPoint(CRGB *pointStart) {
    CRGB *ledGroup = pointStart;
    for (int i = 0; i < NUMBER_GROUPS_PER_CARDINAL_POINT; i++) {
        ledGroup = wrapPointIfNeeded(ledGroup);
        for (int j = 0; j < NUMBER_LEDS_PER_GROUP; j++) {
            ledGroup[j] = POINT_COLOR;
        }
    }
}

static void highlightDirection(float degrees, CRGB *northStart) {
    while (degrees > 360) {
        degrees = degrees - 360;
    }

    // Fill all black...unless we have to walk everything then i can do it in the walk
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
    
    // Which cardinal direction are we closest to? (starting north, going clockwise)
    int offsetLocation = 0;
    for (float cardinalDirection = 0; cardinalDirection < 360; cardinalDirection = cardinalDirection + 90) {
        // Special case north
        float minCardinal = cardinalDirection - DEGREE_VARIATION_FOR_CARDINAL_POINT;
        float maxCardinal = cardinalDirection + DEGREE_VARIATION_FOR_CARDINAL_POINT;
        if (minCardinal < 0) {
            minCardinal += 360;
            if (degrees >= minCardinal && degrees < 360) {
                break; // North
            } else if (degrees >= 0 && degrees < maxCardinal) {
                break; // North
            }
        } else {
            if (degrees >= minCardinal && degrees < maxCardinal) {
                break;
            }
        }
        offsetLocation++;
    }
#if DEBUG
    Serial.print("highlight cardinal group ");
    switch (offsetLocation) {
        case 0: {
            Serial.println("north");
            break;
        }
        case 1: {
            Serial.println("east");
            break;
        }
        case 2: {
            Serial.println("south");
            break;
        }
        case 3: {
            Serial.println("west");
            break;
        }
        default: {
            Serial.println("other");
            break;
        }

    }
#endif
    
    // If less then 4, then we found a cardinal group
    if (offsetLocation < 4) {
        int offset = NUMBER_GROUPS_PER_CARDINAL_POINT*NUMBER_LEDS_PER_GROUP*offsetLocation;
        highlightCardinalPoint(&northStart[offset]);
    } else {
        // We have to do more...highlight more specific things
    }
    
//
//    } else {
//#if DEBUG
//        Serial.println("highlight somethign else");
//#endif
//
//    }
    
    
    
    
}

void initializeBall() {
    
}

// Called from BigBalls.cpp
// TODO: pass the vector to highlight and NOT the degrees, and our standard known vector...
void doDirectionalPoint(float degrees) {
#if DEBUG
    Serial.printf("doDirectionalPoint(%f)\r\n", degrees);
#endif
    
    // TODO: remapping
    // standard north start is the 4th group (after up), see png
    CRGB *northStart = &g_LEDs[4*NUMBER_LEDS_PER_GROUP];
    highlightDirection(degrees, northStart);
    
    
    // We have to show
    FastLED.show();

}
