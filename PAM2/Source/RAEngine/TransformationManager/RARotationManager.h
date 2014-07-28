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
#include "Vec2i.h"
#include "TouchScreenGestureStates.h"

namespace RAEngine
{
    class RARotationManager
    {
    public:
        RARotationManager();
        CGLA::Mat4x4f getRotationMatrix() const;
        void handlePanGesture(GestureState state, CGLA::Vec2i glTouch, CGLA::Vec3f pivotPoint);
        void handleRotationGesture(GestureState state, float rotation, CGLA::Vec3f pivotPoint);
        void reset();
    private:
        CGLA::Vec2i lastLoc;
        float lastRot;
        CGLA::Mat4x4f manualRotationMatrix;
    };
}

#endif /* defined(__PAM2__RARotationManager__) */
