//
//  RABoundingBox.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-23.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RABoundingBox__
#define __PAM2__RABoundingBox__

#include <iostream>
#include "Vec3f.h"

namespace RAEngine {

    class RABoundingBox
    {
    public:
        CGLA::Vec3f minBound;
        CGLA::Vec3f maxBound;
        CGLA::Vec3f center;
        
        float radius;
        float width;
        float height;
        float depth;
    };
}

#endif /* defined(__PAM2__RABoundingBox__) */
