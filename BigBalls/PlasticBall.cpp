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
#if 0 // corbin DEBUG
    #define NUMBER_LEDS_PER_PENTAGON (1) // corbin testing!! so I can use a small strip
#else
    #define NUMBER_LEDS_PER_PENTAGON (6+8+6+8)
#endif

#define PENTAGONS_PER_CIRCLE 8 // Among a 360 view of the ball we will encounter 8 pentagons.
#define PENTAGONS_PER_POLE 4 // Among a top/bottom polar view there are 4 pentagons
#define PENTAGON_COUNT 24

#define POINT_COLOR CRGB::Green // The color we use when pointing on a pentagon

// If this is 1, we will do a test on the start and highlight each pentagon in order so you can verify it
#define HIGHLIGHT_PENTAGONS_ON_START 1

#define DISTANCE_TO_HIGHLIGHT 1.0 // IF LESS THAN THIS value in distance, we will highilght that pentagon. I see values from near 0 (right touching) to ~14 (other side of the ball)

// Orienient the ball with the 1-4 set pointing directly north and the up set pointing directly up. Then set this to one to have it print the current reversed quaterion to account for the offset chip location
#define PRINT_OFFSET_QUAT 1

// min and max are a value between 0 to less than 360. It includes 0 and excludes 360.
typedef struct {
    imu::Vector<3> centerVector;
    CRGB *groupStartLEDs; // Access the NUMBER_LEDS_PER_PENTAGON of LEDs for a pentagon
#if DEBUG
    int offset; // For debugging
#endif
} BallPentagon;

static BallPentagon g_ballPentagons[PENTAGON_COUNT];

// Returns a coordinate in the x/y plane (z=0) for a particular degree
static imu::Vector<3> vectorFromDegrees(float degrees) {
    float rad = radians(degrees);
    return imu::Vector<3>(sin(rad), cos(rad), 0);
}

#if DEBUG

static void printVector(imu::Vector<3> b) {
    Serial.printf("x: %.3f\ty: %.3f\t z:%.3f mag:%.3f\r\n", b.x(), b.y(), b.z(), b.magnitude());
}

static void printPentagon(BallPentagon *ballPentagon) {
    printVector(ballPentagon->centerVector);
}

static void printQuat(const imu::Quaternion &quat) {
    Serial.printf("x: %.3f, y: %.3.f z: %.3f, w: %.3f\r\n", (float)quat.x(), (float)quat.y(), (float)quat.z(), (float)quat.w());
}

#endif

// Find the distance that the end points of the vectors have from each other
static float distanceFromTwoPoints(imu::Vector<3> v1, imu::Vector<3> v2) {
    float x2 = sq(v1.x() - v2.x());
    float y2 = sq(v1.y() - v2.y());
    float z2 = sq(v1.z() - v2.z());
    return sq(x2 + y2 + z2);
}


static void highlightPentagon(BallPentagon *pentagon) {
    // Green direction pointing color??
    fill_solid(pentagon->groupStartLEDs, NUMBER_LEDS_PER_PENTAGON, POINT_COLOR);
}

static void highlightPentagonsNearVector(imu::Vector<3> c) {

    BallPentagon *bestResult = NULL;
    float bestDistance = HUGE_VALF; // large number
    
    for (int i = 0; i < PENTAGON_COUNT; i++) {
        BallPentagon *pentagon = &g_ballPentagons[i];
        // How far is the center of the pentagon from the target location?
        float distance = distanceFromTwoPoints(pentagon->centerVector, c);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestResult = pentagon;
        }
        if (distance < DISTANCE_TO_HIGHLIGHT) {
            highlightPentagon(pentagon);
#if DEBUG
            Serial.printf("      Highlight pentagon model: %d\r\n", i+1);
            //        Serial.printf("Pent %d, dist: %f\r\n", i, distance);
#endif
        }
    }
    if (bestResult == NULL) {
#if DEBUG
        // Shouldn't happen..
        Serial.printf("Couldn't find a pentagon!?? programming error! -> ");
        printVector(c);
        delay(5000);
#endif
        g_patterns.flashThreeTimes(CRGB::Red);
    } else {
#if DEBUG
        Serial.printf("  findPentagonForVector:");
        printVector(c);
        Serial.printf("  Result pentagon sketchup model # %d\r\n----------\r\n", bestResult->offset+1);

#endif
    }
}

#if HIGHLIGHT_PENTAGONS_ON_START

static void highlightPentagonsInOrder() {
    for (int i = 0; i < PENTAGON_COUNT; i++) {
#if DEBUG
        Serial.printf("Pentagon %d (sketchup model #%d)\r\n", i, i+1);
        printPentagon(&g_ballPentagons[i]);
        Serial.println("------");
        Serial.flush();
#endif
        fill_solid(g_LEDs, g_patterns.getLEDCount(), CRGB::Black);

        for (int walk = 0; walk < 3; walk++) {
            fill_solid(g_ballPentagons[i].groupStartLEDs, NUMBER_LEDS_PER_PENTAGON, CRGB::Red);
            FastLED.show();
            delay(150);
            fill_solid(g_ballPentagons[i].groupStartLEDs, NUMBER_LEDS_PER_PENTAGON, CRGB::Black);
            FastLED.show();
            delay(150);
        }
        
        delay(1000);
    }
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

    // Walk the ball along the x/y axis and assign a vector to the center of each pentagon.
    
    // For each cardinal point, assign values to each pentagon that is close to the cardinal directions
    for (float angle = 0; angle < 2*PI; angle = angle + PI/2.0) {
        for (int z = 0; z < 2; z++) {
            // Generate a quaterion to rotate along the z-axis; first do to the left of the angle, and then to the right.
            // Each pentagon lies on about a 45 degree angle for the outer edge. Fake a center based on that.
            // 45 degree = pi/4
            // half of that is roughly the center...
            const float centerOffsetAngle = (PI/4.0)/2.0; // 22.5 degrees
            float zOffsetAngle = z == 0 ? centerOffsetAngle : -centerOffsetAngle; // -1 makes it clockwise
            float centerAngle = -1*angle + zOffsetAngle; // -1 makes it clockwise
            imu::Quaternion zAxisRotationQuat;
            imu::Vector<3> zAxisVector = imu::Vector<3>(0, 0, 1);
            zAxisRotationQuat.fromAxisAngle(zAxisVector, centerAngle); // rotates us along the circle when viewed from above by this angle

            for (int x = 0; x < 2; x++) {
                // Generate a quaterion to rotate "centerOffsetAngle" along the x-axis
                imu::Quaternion xAxisRotationQuat;
                imu::Vector<3> xAxisVector = imu::Vector<3>(1, 0, 0);
                // The new ball layout from M&M is "up, down, down, up";
                float xAxisAngle;
                if (x == 0) {
                    if (z == 0) {
                        xAxisAngle = centerOffsetAngle;
                    } else {
                        xAxisAngle = -centerOffsetAngle;
                    }
                } else {
                    if (z == 0) {
                        xAxisAngle = -centerOffsetAngle;
                    } else {
                        zAxisVector = centerOffsetAngle;
                    }
                }
                
                xAxisRotationQuat.fromAxisAngle(xAxisVector, xAxisAngle); // rotates us up or down by this amount
                
                // Start with a vector pointing north (y=1)
                imu::Vector<3> northVector = imu::Vector<3>(0, 1, 0);
                // Rotate the north vector by this quaterion alnog the x-axis
                imu::Vector<3> firstVector = xAxisRotationQuat.rotateVector(northVector);
                // Rotate the result along the z acox (along a circle when viewed from above)
                imu::Vector<3> resultVector = zAxisRotationQuat.rotateVector(firstVector);

                g_ballPentagons[ballOffset].centerVector = resultVector;
                g_ballPentagons[ballOffset].groupStartLEDs = ledArrayOffset;
#if DEBUG
                g_ballPentagons[ballOffset].offset = ballOffset;
                Serial.printf("sketch: %d \t", ballOffset+1);
                printPentagon(&g_ballPentagons[ballOffset]);
#endif
                ledArrayOffset += NUMBER_LEDS_PER_PENTAGON;
                ballOffset++;
#if DEBUG
//                Serial.printf("angle %.3f - v: ", degrees(angle));
//                printVector(resultVector);
                Serial.flush();
                delay(100); /// not sure why this is needed to see results...
#endif
            }
        }
    }
    
    // Now do the top and bottom set of pentagons; this is practically the same, but we just rotate among the x-axis further up (closer to z=1, but at the center point of the pentagons), and the offset along the z-axis by 45 degrees, which is where their center is located.
    for (int x = 0; x < 2; x++) {
        // The x-axis rotation rotates us up/down
        imu::Quaternion xAxisRotationQuat;
        imu::Vector<3> xAxisVector = imu::Vector<3>(1, 0, 0);
        // Go "up" the first pass, and "down" the second
        const float centerOffsetAngle = 3.0*PI/8.0; // 22.5 degrees from the TOP! (or bottom); 67.5
        float xAxisAngle = x == 0 ? centerOffsetAngle : -centerOffsetAngle;
        xAxisRotationQuat.fromAxisAngle(xAxisVector, xAxisAngle); // rotates us up or down by this amount

        for (float angle = 0; angle < 2*PI; angle = angle + PI/2.0) {
            // The center of this one is simple; it is 45 degrees (pi/4) from our offset "angle"
            float tempAngle = angle - PI/4.0; // Starting point
            float centerAngle = -1*(tempAngle); // -1 makes it clockwise
            imu::Quaternion zAxisRotationQuat;
            imu::Vector<3> zAxisVector = imu::Vector<3>(0, 0, 1);
            zAxisRotationQuat.fromAxisAngle(zAxisVector, centerAngle); // rotates us along the circle when viewed from above by this angle
            // Start with a vector pointing north (y=1)
            imu::Vector<3> northVector = imu::Vector<3>(0, 1, 0);
            // Rotate the north vector by this quaterion alnog the x-axis
            imu::Vector<3> firstVector = xAxisRotationQuat.rotateVector(northVector);
            // Rotate the result along the z acox (along a circle when viewed from above)
            imu::Vector<3> resultVector = zAxisRotationQuat.rotateVector(firstVector);
            
            g_ballPentagons[ballOffset].centerVector = resultVector;
            g_ballPentagons[ballOffset].groupStartLEDs = ledArrayOffset;
#if DEBUG
            g_ballPentagons[ballOffset].offset = ballOffset;
            Serial.printf("%s sketch: %d \t", x==0 ? "top" : "Bottom", ballOffset+1);
            printPentagon(&g_ballPentagons[ballOffset]);
            delay(100); /// not sure why this is needed to see results...
#endif
            ledArrayOffset += NUMBER_LEDS_PER_PENTAGON;
            ballOffset++;
        }
    }
   
#if DEBUG
    if (ballOffset != PENTAGON_COUNT) {
        Serial.printf("ERROR ball count!!!, got %d, expected %d\r\n", ballOffset, PENTAGON_COUNT);
        delay(10000);
    }
    
#if 0 // prints pentagon offsets
    for (int i = 0; i < PENTAGON_COUNT; i++) {
        Serial.printf("Pentagon %d (sketchup model #%d)\r\n", i, i+1);
        printPentagon(&g_ballPentagons[i]);
        Serial.println("------");
        Serial.flush();
        delay(100);
    }
#endif

#if 0
    Serial.println("North");
    printVector(vectorFromDegrees(0));

    Serial.println("NE");
    printVector(vectorFromDegrees(45));

    Serial.println("East");
    printVector(vectorFromDegrees(90));

    Serial.println("South");
    printVector(vectorFromDegrees(180));

    Serial.println("West");
    printVector(vectorFromDegrees(270));
#endif
    
    Serial.println("-- Short wait to see results ----");
    Serial.flush();

//    delay(5000);
    
#endif
    
    
#if HIGHLIGHT_PENTAGONS_ON_START
    highlightPentagonsInOrder();
#endif
}

// Called from BigBalls.cpp
void doDirectionalPointWithOrientation(float targetDirectionInDegrees) {
    targetDirectionInDegrees = 0;
    imu::Quaternion orientationQuat = g_bno.getQuat();
    imu::Quaternion reverseRotationQuat = orientationQuat.conjugate();
    
    // This value should be set to what we print
    imu::Quaternion offsetQuat = imu::Quaternion(1.0, 0.0, 0.0, 0.0);
//    imu::Quaternion offsetQuat = imu::Quaternion(0.162842, -0.331543, -0.621033, -0.691284);
#if PRINT_OFFSET_QUAT || DEBUG
    Serial.printf("imu::Quaternion offsetQuat = imu::Quaternion(%f, %f, %f, %f);\r\n", (float)reverseRotationQuat.w(), (float)reverseRotationQuat.x(), (float)reverseRotationQuat.y(), (float)reverseRotationQuat.z());
    
#endif

    // Take the z axis and unrotate it by the amount that the BNO has been rotated. This will get it pointing up.
    imu::Vector<3> zAxisVector = imu::Vector<3>(0, 0, 1);
    zAxisVector = offsetQuat.rotateVector(zAxisVector);

    zAxisVector = reverseRotationQuat.rotateVector(zAxisVector);
    // This should always point up on the ball, regardless of its orientation. We use that as our new z-axis, and rotate a vector from north pointed to the way we want to go
    // Start with a vector pointing north (y=1)
    imu::Vector<3> northVector = imu::Vector<3>(0, 1, 0);
    northVector = offsetQuat.rotateVector(northVector);
    
    // reverse it so we have a vector pointing towards north on the ball
    northVector = reverseRotationQuat.rotateVector(northVector);
    // Now, we rotate that vector to point towards the tower, first by generating a quaterion for that rotation
    imu::Quaternion rotationToTowerQuat;
    // negative, to rotate clockwise
    rotationToTowerQuat.fromAxisAngle(zAxisVector, -radians(targetDirectionInDegrees));
    imu::Vector<3> targetVector = rotationToTowerQuat.rotateVector(northVector);
    
#if DEBUG
    Serial.printf("Target vector direction for %f degrees: ", targetDirectionInDegrees);
    printVector(targetVector);
#endif
    
    // Fill all black first
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
    highlightPentagonsNearVector(targetVector);
    
    // We have to show, as we aren't using the patterns library and just using FastLED
    FastLED.show();

}
