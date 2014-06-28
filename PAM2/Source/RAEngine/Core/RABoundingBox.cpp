//
//  RABoundingBox.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-23.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RABoundingBox.h"

namespace RAEngine {
    
    using namespace CGLA;
    
    RABoundingBox::RABoundingBox() : RABoundingBox(Vec3f(0,0,0), Vec3f(1,1,1))
    {
    };

    RABoundingBox::RABoundingBox(CGLA::Vec3f pmin, CGLA::Vec3f pmax)
    {
        minBound = pmin;
        maxBound = pmax;
        
        Vec3f mid = maxBound - minBound;
        radius = 0.5*mid.length();
        center =  minBound + radius*normalize(maxBound);
        
        width = fabsf(maxBound[0] - minBound[0]);
        height = fabsf(maxBound[1] - minBound[1]);
        depth = fabsf(maxBound[2] - minBound[2]);
    }
}