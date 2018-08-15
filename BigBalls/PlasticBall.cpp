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
    #define NUMBER_LEDS_PER_PENTAGON (8+6+8+6)
#endif

#define PENTAGONS_PER_CIRCLE 8 // Among a 360 view of the ball we will encounter 8 pentagons.
#define PENTAGONS_PER_POLE 4 // Among a top/bottom polar view there are 4 pentagons
#define PENTAGON_COUNT 24

#define POINT_COLOR CRGB::Green


#define DEGREE_VARIATION_FOR_CARDINAL_POINT 10 // if we are within <value> degrees from a cardinal point, we will highlight all four pentagons. Otherwise, we higlight two (or maybe one)

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

static BallPentagon *findPentagonForVector(imu::Vector<3> c) {
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
    return bestResult;
}

static void hilightPentagon(BallPentagon *pentagon) {
    // Green direction pointing color??
    fill_solid(pentagon->groupStartLEDs, NUMBER_LEDS_PER_PENTAGON, CRGB::Green);
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
                // Go "up" the first pass, and "down" the second
                float xAxisAngle = x == 0 ? centerOffsetAngle : -centerOffsetAngle;
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
}

// Called from BigBalls.cpp
void doDirectionalPointWithOrientation(float targetDirectionInDegrees) {
    imu::Quaternion orientationQuat = g_bno.getQuat();
#if  0 // DEBUG
    imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Serial.printf("TEST doDirectionalPointWithOrientation(%f)\r\n", targetDirectionInDegrees);
    
    printQuat(orientationQuat);
    
    Serial.printf("degrees %f -> ", targetDirectionInDegrees);
    Serial.println(TinyGPSPlus::cardinal(targetDirectionInDegrees));
    
    
    /* Also send calibration data for each sensor. */
    uint8_t sys, gyro, accel, mag = 0;
    g_bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.println(mag, DEC);
    
#endif
    // Fill all black first
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);

    // Take the z axis and unrotate it by the amount that the BNO has been rotated. This will get it pointing up.
    imu::Vector<3> zAxisVector = imu::Vector<3>(0, 0, 1);
    imu::Quaternion reverseRotationQuat = orientationQuat.conjugate();
    zAxisVector = reverseRotationQuat.rotateVector(zAxisVector);
    // This should always point up on the ball, regardless of its orientation. We use that as our new z-axis, and rotate
    
    
    
    // hack
   // targetDirectionInDegrees = 0; // for testing go north
//    orientationQuat = imu::Quaternion();
//    imu::Vector<3> xAxisVector = imu::Vector<3>(1, 0, 0);
//    imu::Quaternion zAxisRotationQuat;
//    zAxisRotationQuat.fromAxisAngle(zAxisVector, -(PI/2+PI/8)); // rotates us along the circle when viewed
//    orientationQuat.fromAxisAngle(xAxisVector, (PI/2 - PI/8));
    // end hack
    
//    imu::Vector<3> coordinateForDirection = vectorFromDegrees(targetDirectionInDegrees);
//    // Now rotate it
//    imu::Vector<3> rotatedVector = orientationQuat.rotateVector(coordinateForDirection);
//
////    rotatedVector = zAxisRotationQuat.rotateVector(rotatedVector);
//
//    printQuat(orientationQuat);
//    imu::Vector<3> euler = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    Serial.printf("Euler x:%.2f y:%.2f z:%.2f\r\n", euler.x(), euler.y(), euler.z());
//    euler = orientationQuat.toEuler();
//    Serial.printf("Euler2 x:%.2f y:%.2f z:%.2f\r\n", euler.x(), euler.y(), euler.z());


    BallPentagon *pentagon = findPentagonForVector(zAxisVector);
    if (pentagon == NULL) return; // Avoid crashing in case something is wrong with the code
    hilightPentagon(pentagon);
    
    // We have to show, as we aren't using the patterns library and just using FastLED
    FastLED.show();

}
