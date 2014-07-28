//
//  RotationManager.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-18.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RARotationManager.h"

namespace RAEngine
{
    using namespace CGLA;
    
    RARotationManager::RARotationManager()
    {
        manualRotationMatrix = identity_Mat4x4f();
    }
    
    void RARotationManager::handlePanGesture(GestureState state, CGLA::Vec2i glTouch, CGLA::Vec3f pivotPoint)
    {
        if (state == GestureState::Began) {
            lastLoc = glTouch;
        } else if (state == GestureState::Changed) {
            Vec2i diff = glTouch - lastLoc;
            float rotX =  (diff[1]/2.0f) * DEGREES_TO_RADIANS; //because positive angle is clockwise
            float rotY =  -1*(diff[0]/2.0f) * DEGREES_TO_RADIANS;
            
            Mat4x4f curModelViewInverse = invert_ortho(manualRotationMatrix);
            Vec3f xAxis = Vec3f(curModelViewInverse * Vec4f(1, 0, 0, 0));
            Vec3f yAxis = Vec3f(curModelViewInverse * Vec4f(0, 1, 0, 0));
            Mat4x4f rm = rotation_Mat4x4f(xAxis, rotX);
            Mat4x4f rm2 = rotation_Mat4x4f(yAxis, rotY);
            Mat4x4f toOrigin = translation_Mat4x4f(pivotPoint);
            Mat4x4f fromOrigin = translation_Mat4x4f(-1*pivotPoint);
            manualRotationMatrix = manualRotationMatrix * toOrigin * rm * rm2 * fromOrigin;
            
            lastLoc = glTouch;
        }
    }
    
    void RARotationManager::handleRotationGesture(GestureState state, float rotation, CGLA::Vec3f pivotPoint)
    {
        if (state == GestureState::Began) {
            lastRot = rotation;
        } else if (state == GestureState::Changed) {
            float rotZ = -1 * (rotation - lastRot);

            Vec3f zAxis = Vec3f(invert_ortho(manualRotationMatrix) * Vec4f(0,0,-1,0));
            Mat4x4f rm = rotation_Mat4x4f(zAxis, rotZ);
            Mat4x4f toOrigin = translation_Mat4x4f(pivotPoint);
            Mat4x4f fromOrigin = translation_Mat4x4f(-1*pivotPoint);
            manualRotationMatrix = manualRotationMatrix * toOrigin * rm * fromOrigin;
            lastRot = rotation;
        }
    }

    CGLA::Mat4x4f RARotationManager::getRotationMatrix() const
    {
        return manualRotationMatrix;
    }
    
    void RARotationManager::reset()
    {
        manualRotationMatrix = identity_Mat4x4f();
    }
    
}