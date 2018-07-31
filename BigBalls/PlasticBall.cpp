//
//  PlasticBall.cpp
//  Index
//
//  Created by Corbin Dunn on 7/30/18 .
//

#include "PlasticBall.h"
#include "BigBalls.h"

// Things specific to the plastic ball

typedef CD_ENUM(int16_t, CDBallPoint)  {
    CDBallPointUp,
    CDBallPointNorth,
    CDBallPointEast,
    CDBallPointSouth,
    CDBallPointWest,
    CDBallPointDown
};

// Called from BigBalls.cpp
void doDirectionalPoint() {
#if DEBUG
    
#endif
    // TODO: calculate direction...and update stuff here...
    
    
    g_patterns.setPatternType(LEDPatternTypeSolidColor);
    g_patterns.setPatternColor(CRGB::Green);

}
