//
//  RotationManager.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-18.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RARotationManager.h"
#import "Quatf.h"


namespace RAEngine {
    using namespace CGLA;
    RARotationManager::RARotationManager()
    {
        manualRotationMatrix = identity_Mat4x4f();
    }
    
    void RARotationManager::handlePanGesture(UIGestureRecognizer* sender)
    {

        UIPanGestureRecognizer* pan = (UIPanGestureRecognizer*)sender;
        CGPoint cocoaTouch = [pan locationInView:sender.view];
        CGPoint glTouch = CGPointMake(cocoaTouch.x, [pan.view bounds].size.height - cocoaTouch.y);
        
        if (pan.state == UIGestureRecognizerStateBegan) {
            lastLoc = glTouch;
        } else if (pan.state == UIGestureRecognizerStateChanged) {
            CGPoint diff = CGPointMake(glTouch.x - lastLoc.x, glTouch.y - lastLoc.y);
            float rotX =  -1*(diff.y / 2.0f)*DEGREES_TO_RADIANS; //because positive angle is clockwise
            float rotY =  (diff.x / 2.0f) *DEGREES_TO_RADIANS;

            Mat4x4f curModelViewInverse = invert_ortho(manualRotationMatrix);
            Vec3f xAxis = Vec3f(curModelViewInverse * Vec4f(1, 0, 0, 0));
            Vec3f yAxis = Vec3f(curModelViewInverse * Vec4f(0, 1, 0, 0));
            Mat4x4f rm = rotation_Mat4x4f(xAxis, rotX);
            Mat4x4f rm2 = rotation_Mat4x4f(yAxis, rotY);
            manualRotationMatrix = manualRotationMatrix * rm * rm2;

            lastLoc = glTouch;
        }
    }

    CGLA::Mat4x4f RARotationManager::getRotationMatrix() const
    {
        return manualRotationMatrix;
    }
    
}