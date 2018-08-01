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
#if X // DEBUG
    #define NUMBER_LEDS_PER_GROUP (1) // corbin testing!! so I can use a small strip
#else
    #define NUMBER_LEDS_PER_GROUP (8+6+8+6)
#endif

#define POINT_COLOR CRGB::Green

#define NUMBER_GROUPS_PER_CARDINAL_POINT 4 // four pentagons per cardinal point/direction

#define DEGREE_VARIATION_FOR_CARDINAL_POINT 10 // if we are within <value> degrees from a cardinal point, we will highlight all four pentagons. Otherwise, we higlight two (or maybe one)

typedef CD_ENUM(int16_t, CDCardinalDirection) {
    CDCardinalDirectionNorth = 0,
    CDCardinalDirectionEast,
    CDCardinalDirectionSouth,
    CDCardinalDirectionWest,
    CDCardinalDirectionUnknown
};

#if DEBUG
static char *c_cardinalDirectionNames[] = {
    "North", "East", "South", "West", "Unknown"
};
#endif


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

static CDCardinalDirection computeClosestCardinalDirectionFromDegrees(float degrees, float offsetDegrees) {
    while (degrees >= 360.0) {
        degrees = degrees - 360.0;
    }
        // Which cardinal direction are we closest to? (starting north, going clockwise)
    CDCardinalDirection cardinalDirection = CDCardinalDirectionNorth;
    for (float compassValue = 0; compassValue < 360.0; compassValue = compassValue + 90.0) {
        // Special case north
        float minCardinal = compassValue - offsetDegrees;
        float maxCardinal = compassValue + offsetDegrees;
        if (minCardinal < 0.0) {
            minCardinal += 360.0;
            if (degrees >= minCardinal && degrees < 360.0) {
                break; // North
            } else if (degrees >= 0.0 && degrees < maxCardinal) {
                break; // North
            }
        } else {
            if (degrees >= minCardinal && degrees < maxCardinal) {
                break;
            }
        }
        cardinalDirection = static_cast<CDCardinalDirection>((int)cardinalDirection + 1);
    }
    return cardinalDirection;
}

static void highlightDirection(float degrees, CRGB *northStart) {
    // Fill all black...unless we have to walk everything then i can do it in the walk
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);

    // For ease of use, highlight all four in the main cardinal direction we are pointing
    CDCardinalDirection cardinalDirection = computeClosestCardinalDirectionFromDegrees(degrees, 45);
    
#if DEBUG
    Serial.printf("degrees %f: highlight cardinal group %s (value: %d)\r\n", degrees, c_cardinalDirectionNames[cardinalDirection], cardinalDirection);
#endif
    
    if (cardinalDirection < CDCardinalDirectionUnknown) {
        int offset = NUMBER_GROUPS_PER_CARDINAL_POINT*NUMBER_LEDS_PER_GROUP*cardinalDirection;
        highlightCardinalPoint(&northStart[offset]);
    } else {
        // We shouldn't hit this..
        g_patterns.flashThreeTimes(CRGB::Red);
#if DEBUG
        Serial.printf("Error computing cardinal direction for %f\r\n", degrees);
//        delay(10000); // debug value 360 that is failing
#endif
    }
    
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
