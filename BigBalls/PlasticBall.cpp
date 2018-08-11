//
//  PlasticBall.cpp
//  Index
//
//  Created by Corbin Dunn on 7/30/18 .
//

#include "PlasticBall.h"
#include "BigBalls.h"
#include "HardwareSerial.h"
#include "TinyGPS++.h"

// Things specific to the plastic ball

// These are defined in BigBalls.h, but probably should be hidden and only in this location
#if 0 // 1 // corbin DEBUG
    #define NUMBER_LEDS_PER_PENTAGON (1) // corbin testing!! so I can use a small strip
#else
    #define NUMBER_LEDS_PER_PENTAGON (8+6+8+6)
#endif

#define PENTAGONS_PER_CIRCLE 8 // Among a 360 view of the ball we will encounter 8 pentagons.
#define PENTAGONS_PER_POLE 4 // Among a top/bottom polar view there are 4 pentagons
#define PENTAGON_COUNT 24

#define POINT_COLOR CRGB::Green


#define DEGREE_VARIATION_FOR_CARDINAL_POINT 10 // if we are within <value> degrees from a cardinal point, we will highlight all four pentagons. Otherwise, we higlight two (or maybe one)

// min and max are a value between 0 to less than 360. It includes 0 and excludes 360.
typedef struct {
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    CRGB *groupStartLEDs; // Access the NUMBER_LEDS_PER_PENTAGON of LEDs for a pentagon
#if DEBUG
    int offset; // For debugging
#endif
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

static void printPentagon(BallPentagon *ballPentagon) {
    Serial.printf("minX: %.3f\t maxX: %.3f\r\n", ballPentagon->minX, ballPentagon->maxX);
    Serial.printf("minY: %.3f\t maxY: %.3f\r\n", ballPentagon->minY, ballPentagon->maxY);
    Serial.printf("minZ: %.3f\t maxZ: %.3f\r\n", ballPentagon->minZ, ballPentagon->maxZ);
}

#endif


static BallPentagon *findPentagonForCoordinate(BallCoordinate c) {
    // Some sanity validations
    if (abs(c.x) > 1.0 || abs(c.y) > 1.0 || abs(c.z) > 1.0) {
#if DEBUG
        Serial.printf("Out of bounds coordinate! -> ");
        printBallCoordiante(c);
#endif
        g_patterns.flashThreeTimes(CRGB::Red);
    }
    for (int i = 0; i < PENTAGON_COUNT; i++) {
        BallPentagon *pentagon = &g_ballPentagons[i];
        if (c.x >= pentagon->minX && c.x <= pentagon->maxX &&
            c.y >= pentagon->minY && c.y <= pentagon->maxY &&
            c.z >= pentagon->minZ && c.z <= pentagon->maxZ) {
            return pentagon; // Found it!
        }
    }
#if DEBUG
    // Shouldn't happen..
    Serial.printf("Couldn't find a pentagon!?? programming error! -> ");
    printBallCoordiante(c);
    delay(5000);
#endif
    g_patterns.flashThreeTimes(CRGB::Red);
    return NULL;
}

static void hilightPentagon(BallPentagon *pentagon) {
    // Green direction pointing color??
    fill_solid(pentagon->groupStartLEDs, NUMBER_LEDS_PER_PENTAGON, CRGB::Green);
    
#if DEBUG
    Serial.printf("Highlight pentagon offset: %d (sketchup model # %d)\r\n----------\r\n", pentagon->offset, pentagon->offset+1);
#endif
}



void initializeBall() {
    // Figure out the coordinate min/max points along the sphere for each pentagon. We'll highlight a pentagon if it is present in this area
    
    // corbin remember:
    // cos(0) = 1
    // sin(0) = 0
    
    // Start at the start of the array for north, incremented as we fill the ball pentagon array
    CRGB *ledArrayOffset = g_LEDs;
    // We fill up the ball pentagon array by incrementing this offset
    int ballOffset = 0;

#define SMALL_OFFSET 0 // WE MIGHT need to make this a small number close to 0, like 0.001, in order to avoid rounding issues
    
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
                g_ballPentagons[ballOffset].minX = MIN(minX, maxX) - SMALL_OFFSET;
                g_ballPentagons[ballOffset].maxX = MAX(minX, maxX) + SMALL_OFFSET;
                g_ballPentagons[ballOffset].minY = MIN(minY, maxY) - SMALL_OFFSET;
                g_ballPentagons[ballOffset].maxY = MAX(minY, maxY) + SMALL_OFFSET;
                g_ballPentagons[ballOffset].minZ = minZ - SMALL_OFFSET;
                g_ballPentagons[ballOffset].maxZ = maxZ + SMALL_OFFSET;
                g_ballPentagons[ballOffset].groupStartLEDs = ledArrayOffset;
                g_ballPentagons[ballOffset].offset = ballOffset;

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
                
                g_ballPentagons[ballOffset].minX = MIN(minX, maxX) - SMALL_OFFSET;
                g_ballPentagons[ballOffset].maxX = MAX(minX, maxX) + SMALL_OFFSET;
                g_ballPentagons[ballOffset].minY = MIN(minY, maxY) - SMALL_OFFSET;
                g_ballPentagons[ballOffset].maxY = MAX(minY, maxY) + SMALL_OFFSET;
                g_ballPentagons[ballOffset].minZ = minZ - SMALL_OFFSET;
                g_ballPentagons[ballOffset].maxZ = maxZ + SMALL_OFFSET;

                g_ballPentagons[ballOffset].groupStartLEDs = ledArrayOffset;
                g_ballPentagons[ballOffset].offset = ballOffset;
                
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
    
#if 0 // prints pentagon offsets
    for (int i = 0; i < PENTAGON_COUNT; i++) {
        Serial.printf("Pentagon %d\r\n", i);
        printPentagon(&g_ballPentagons[i]);
        Serial.println("------");
        Serial.flush();
        delay(100);
    }
#endif

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
    
#if 0
    // test finding the pentagons....
    for (float i = 0; i < 360; i += 45) {
        BallCoordinate b = ballCoordinateFromDegrees(i);
        b.z = -.04;
        BallPentagon *pentagon = findPentagonForCoordinate(b);
        hilightPentagon(pentagon);
    }
#endif
    Serial.println("-- Short wait to see results ----");
    Serial.flush();

//    delay(5000);
    
    
#endif
}

// Called from BigBalls.cpp
void doDirectionalPointWithOrientation(float targetDirectionInDegrees, imu::Quaternion orientationQuat) {

#if  DEBUG
    imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /// HACK!! hardcode the degrees based on euler for now
    targetDirectionInDegrees = euler.x();
    
    Serial.printf("TEST doDirectionalPointWithOrientation(.., %f)\r\n", targetDirectionInDegrees);
    
//    Serial.printf("Euler.x: %.2f\r\n", euler.x());
//    Serial.printf("Euler.y: %.2f\r\n", euler.y());
//    Serial.printf("Euler.z: %.2f\r\n", euler.z());
//
    Serial.printf("quaterion x: %.2f\r\n", (float)orientationQuat.x());
    Serial.printf("quaterion y: %.2f\r\n", (float)orientationQuat.y());
    Serial.printf("quaterion z: %.2f\r\n", (float)orientationQuat.z());
    Serial.printf("quaterion w: %.2f\r\n", (float)orientationQuat.w());
    
    Serial.printf("degrees %f -> ", targetDirectionInDegrees);
    Serial.println(TinyGPSPlus::cardinal(targetDirectionInDegrees));
    
#endif
    // Fill all black first
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
    
    BallCoordinate coordinateForDirection = ballCoordinateFromDegrees(targetDirectionInDegrees);

    // TODO: rotate based on orientationQuat....!
    BallPentagon *pentagon = findPentagonForCoordinate(coordinateForDirection);
    if (pentagon == NULL) return; // Avoid crashing in case something is wrong with the code
    hilightPentagon(pentagon);
    
    // We have to show, as we aren't using the patterns library and just using FastLED
    FastLED.show();

}
