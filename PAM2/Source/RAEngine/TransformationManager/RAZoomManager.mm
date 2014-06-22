//
//  RAZoomManager.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-19.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAZoomManager.h"
#include "RALogManager.h"

namespace RAEngine
{
    void RAZoomManager::handlePinchGesture(UIGestureRecognizer* sender)
    {
        UIPinchGestureRecognizer* pinch = (UIPinchGestureRecognizer*) sender;
        if (sender.state == UIGestureRecognizerStateBegan)
        {
            curFactor = scaleFactor;
        } else {
            scaleFactor = curFactor * [pinch scale];
        }
    }
    
    float RAZoomManager::getScaleFactor() const
    {
        if (scaleFactor < 0) {
            RA_LOG_WARN("Negative scalefactor: %f", scaleFactor);
        }
        return scaleFactor;
    }
}