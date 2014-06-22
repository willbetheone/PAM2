//
//  RotationManager.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-18.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RARotationManager__
#define __PAM2__RARotationManager__

#include <iostream>
#include "Mat4x4f.h"
#import <GLKit/GLKMath.h>

namespace RAEngine
{
    class RARotationManager
    {
    public:
        RARotationManager();
        CGLA::Mat4x4f getRotationMatrix() const;
        void handlePanGesture(UIGestureRecognizer* sender);
    private:
        CGPoint lastLoc;
        CGLA::Mat4x4f manualRotationMatrix;
    };
}

#endif /* defined(__PAM2__RARotationManager__) */
