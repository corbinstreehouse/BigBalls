//
//  PlasticBall.cpp
//  Index
//
//  Created by Corbin Dunn on 7/30/18 .
//

#include "PlasticBall.h"
#include "BigBalls.h"
#include "HardwareSerial.h"

// Things specific to the plastic ball

// These are defined in BigBalls.h, but probably should be hidden and only in this location
#if 0 // corbin DEBUG
    #define NUMBER_LEDS_PER_PENTAGON (2) // corbin testing!! so I can use a small strip
    #define NUMBER_PENTAGONS_PER_CARDINAL_POINT 1 //// more corbin testing!
#else
    #define NUMBER_LEDS_PER_PENTAGON (8+6+8+6)
    #define NUMBER_PENTAGONS_PER_CARDINAL_POINT 4 // four pentagons per cardinal point/direction
#endif

#define PENTAGONS_PER_CIRCLE 8 // Among a 360 view of the ball we will encounter 8 pentagons.
#define PENTAGONS_PER_POLE 4 // Among a top/bottom polar view there are 4 pentagons
#define PENTAGON_COUNT 24

#define POINT_COLOR CRGB::Green


#define DEGREE_VARIATION_FOR_CARDINAL_POINT 10 // if we are within <value> degrees from a cardinal point, we will highlight all four pentagons. Otherwise, we higlight two (or maybe one)

typedef CD_ENUM(int16_t, CDCardinalDirection) {
    CDCardinalDirectionNorth = 0,
    CDCardinalDirectionEast,
    CDCardinalDirectionSouth,
    CDCardinalDirectionWest,
    CDCardinalDirectionUnknown
};

// min and max are a value between 0 to less than 360. It includes 0 and excludes 360.
typedef struct {
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    CRGB *groupStartLEDs; // Access the NUMBER_LEDS_PER_PENTAGON of LEDs for a pentagon
} BallPentagon;

static BallPentagon g_ballPentagons[PENTAGON_COUNT];

typedef struct {
    float x, y, z;
} BallCoordinate;

static BallCoordinate makeBallCoordinate(float x, float y, float z) {
    BallCoordinate r;
    r.x = x;
    r.y = y;
    r.z = z;
    return r;
}

#if DEBUG
static char *c_cardinalDirectionNames[] = {
    "North", "East", "South", "West", "Unknown"
};
#endif

// Returns a coordinate in the x/y plane (z=0) for a particular degree
static BallCoordinate ballCoordinateFromDegrees(float degrees) {
    float rad = radians(degrees);
    BallCoordinate result;
    result.z = 0;
    result.x = sin(rad);
    result.y = cos(rad);
    return result;
}

#if DEBUG

static void printBallCoordiante(BallCoordinate b) {
    Serial.printf("x: %.3f\ty: %.3f\t z:%.3f\r\n", b.x, b.y, b.z);
}

#endif

/*
static inline CRGB *wrapPointIfNeeded(CRGB *point) {
    int amountPast = point - g_LEDsPastEnd;
    if (amountPast >= 0) {
        return &g_LEDs[amountPast];
    } else {
        return point;
    }
}

static inline void highlightCardinalPoint(CRGB *pointStart) {
    CRGB *ledGroup = pointStart;
    for (int i = 0; i < NUMBER_PENTAGONS_PER_CARDINAL_POINT; i++) {
        ledGroup = wrapPointIfNeeded(ledGroup);
        for (int j = 0; j < NUMBER_LEDS_PER_PENTAGON; j++) {
            *ledGroup = POINT_COLOR;
            ledGroup++;
        }
    }
}

static CDCardinalDirection computeClosestCardinalDirectionFromDegrees(float degrees, float offsetDegrees) {
    while (degrees >= 360.0) {
        degrees = degrees - 360.0;
    }
    while (degrees < 0.0) {
        degrees += 360.0;
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
        int offset = NUMBER_PENTAGONS_PER_CARDINAL_POINT*NUMBER_LEDS_PER_PENTAGON*cardinalDirection;
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
*/

#if DEBUG

static bool checkRange(float *min, float *max) {
    if (*min >= *max) {
        Serial.printf("RANGE ERROR: min: %.3f, max %.3f\r\n", *min, *max);
        Serial.flush();
        delay(100000);
    }
}
static void printPentagon(BallPentagon *ballPentagon) {
    Serial.printf("minX: %.3f\t maxX: %.3f\r\n", ballPentagon->minX, ballPentagon->maxX);
    checkRange(&ballPentagon->minX, &ballPentagon->maxX);
    Serial.printf("minY: %.3f\t maxY: %.3f\r\n", ballPentagon->minY, ballPentagon->maxY);
    checkRange(&ballPentagon->minY, &ballPentagon->maxY);
    Serial.printf("minZ: %.3f\t maxZ: %.3f\r\n", ballPentagon->minZ, ballPentagon->maxZ);
    checkRange(&ballPentagon->minZ, &ballPentagon->maxZ);
}

#endif


void initializeBall() {
    // Figure out the coordinate min/max points along the sphere for each pentagon. We'll highlight a pentagon if it is present in this area
    
    // corbin remember:
    // cos(0) = 1
    // sin(0) = 0
    
    // Start at the start of the array for north, incremented as we fill the ball pentagon array
    CRGB *ledArrayOffset = g_LEDs;
    // We fill up the ball pentagon array by incrementing this offset
    int ballOffset = 0;

    // For the first "north group" of 4 pentagons we want the sharp corner to point dead north, since there are four cardinal points that we can easily make inside the "ball". So, instead of starting at 0, I'm going to back off the x one level
    {
        // So, walk around the sphere and assign cartisian coordinates to portions that will correspond to where the thing should light up a given pentagon. I could hardcode this more for speed.
        // We use a unit sphere for coordinates.
        const float angleStep = 2*PI / PENTAGONS_PER_CIRCLE; // For 8 pentagons, this is pi/4 (or 45 degrees)
        const float startAngle = -angleStep;
        const float endAngle = TWO_PI - angleStep;
        const float sinAngleStep = sin(angleStep);

        float minX = sin(startAngle);
        float minY = sin(startAngle);
        
        // This will only work for the pentagon layout...I could make it more abstract in the Z
        // First, handle all the pentagons touching the x/y plane
        for (float angle = (startAngle + angleStep); angle <= endAngle; angle += angleStep) {
            // Generate the x and y end point
            float maxX = sin(angle);
            float maxY = cos(angle);

            float minZ = 0.0; // sin(0)
            float maxZ = sinAngleStep;

            // Fill in the top one and then the bottom one
            for (int i = 0; i < 2; i++) {
                g_ballPentagons[ballOffset].minX = MIN(minX, maxX);
                g_ballPentagons[ballOffset].maxX = MAX(minX, maxX);
                g_ballPentagons[ballOffset].minY = MIN(minY, maxY);
                g_ballPentagons[ballOffset].maxY = MAX(minY, maxY);
                g_ballPentagons[ballOffset].minZ = minZ;
                g_ballPentagons[ballOffset].maxZ = maxZ;
                g_ballPentagons[ballOffset].groupStartLEDs = ledArrayOffset;

#if 0 // DEBUG
                Serial.printf("Setup Pentagon %d\r\n", ballOffset);
                printPentagon(&g_ballPentagons[ballOffset]);
                Serial.println("------");
                Serial.flush();
                delay(10);
#endif
                
                ledArrayOffset += NUMBER_LEDS_PER_PENTAGON;
                ballOffset++;

                // Go the other way for the Z
                float oldMinZ = minZ;
                minZ = -maxZ;
                maxZ = oldMinZ;
            }
            // Go to the next one along the x/y plane
            minX = maxX;
            minY = maxY;
        }
    }
        
    // The above handles all the pentagons that touch the x/y plane; now handle the top and bottom pentagon groups
    // This is almost the same...and I could combine the loops, but the code would be harder to read
    {
        // The top/bottom pentagon group has four pentagons (PENTAGONS_PER_POLE)
        const float angleStep = 2*PI / PENTAGONS_PER_POLE;
        const float startAngle = -angleStep;
        const float endAngle = TWO_PI - angleStep;
        const float sinZStart = sin(2*PI / PENTAGONS_PER_CIRCLE);

        for (int i = 0; i < 2; i++) {
            float minX = sin(startAngle);
            float minY = sin(startAngle);

            float minZ, maxZ;
            if (i == 0) {
                minZ = sinZStart;
                maxZ = 1.0; // cos(0)
            } else {
                minZ = -1.0; // cos(180)
                maxZ = -sinZStart;
            }

            // The top/bottom groups of pentagons have four
            static const float angleStep = 2*PI / PENTAGONS_PER_POLE; // For 4 pentagons, this is pi/2 (or 90 degrees)
            for (float angle = (startAngle + angleStep); angle <= endAngle; angle += angleStep) {
                // Generate the x and y end point
                float maxX = sin(angle);
                float maxY = cos(angle);
                
                g_ballPentagons[ballOffset].minX = MIN(minX, maxX);
                g_ballPentagons[ballOffset].maxX = MAX(minX, maxX);
                g_ballPentagons[ballOffset].minY = MIN(minY, maxY);
                g_ballPentagons[ballOffset].maxY = MAX(minY, maxY);
                g_ballPentagons[ballOffset].minZ = minZ;
                g_ballPentagons[ballOffset].maxZ = maxZ;
                g_ballPentagons[ballOffset].groupStartLEDs = ledArrayOffset;
                
                ledArrayOffset += NUMBER_LEDS_PER_PENTAGON;
                ballOffset++;
                
                minX = maxX;
                minY = maxY;
            }
        }
    }
    
#if DEBUG
    if (ballOffset != PENTAGON_COUNT) {
        Serial.printf("ERROR ball count!!!, got %d, expected %d\r\n", ballOffset, PENTAGON_COUNT);
        delay(10000);
    }
    
    for (int i = 0; i < PENTAGON_COUNT; i++) {
        Serial.printf("Pentagon %d\r\n", i);
        printPentagon(&g_ballPentagons[i]);
        Serial.println("------");
        Serial.flush();
        delay(100);
    }
    
    Serial.println("-- Short wait to see results ----");
    Serial.flush();

#if 0
    Serial.println("North");
    printBallCoordiante(ballCoordinateFromDegrees(0));

    Serial.println("NE");
    printBallCoordiante(ballCoordinateFromDegrees(45));

    Serial.println("East");
    printBallCoordiante(ballCoordinateFromDegrees(90));

    Serial.println("South");
    printBallCoordiante(ballCoordinateFromDegrees(180));

    Serial.println("West");
    printBallCoordiante(ballCoordinateFromDegrees(270));
#endif
    
    delay(5000);
    
    
#endif
}

// Called from BigBalls.cpp
void doDirectionalPointWithOrientation(float targetDirectionInDegrees, imu::Quaternion orientation) {

#if  DEBUG
    imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /// HACK!! hardcode the degrees based on euler for now
    targetDirectionInDegrees = euler.x();
    
    Serial.printf("TEST doDirectionalPointWithOrientation(.., %f)\r\n", targetDirectionInDegrees);
    
//    Serial.printf("Euler.x: %.2f\r\n", euler.x());
//    Serial.printf("Euler.y: %.2f\r\n", euler.y());
//    Serial.printf("Euler.z: %.2f\r\n", euler.z());
//
    Serial.printf("quaterion x: %.2f\r\n", (float)orientation.x());
    Serial.printf("quaterion y: %.2f\r\n", (float)orientation.y());
    Serial.printf("quaterion z: %.2f\r\n", (float)orientation.z());
    Serial.printf("quaterion w: %.2f\r\n", (float)orientation.w());

#endif
    // Fill all black first
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
    
    
    

    
    // We have to show, as we aren't using the patterns library and just using FastLED
    FastLED.show();

}
