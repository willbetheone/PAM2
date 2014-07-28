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
    void RAZoomManager::handlePinchGesture(GestureState state, float scale)
    {
        if (state == GestureState::Began) {
            curFactor = scaleFactor;
        } else {
            scaleFactor = curFactor * scale;
        }
    }
    
    float RAZoomManager::getScaleFactor() const
    {
        if (scaleFactor < 0) {
            RA_LOG_WARN("Negative scalefactor: %f", scaleFactor);
        }
        return scaleFactor;
    }
    
    void RAZoomManager::reset()
    {
        scaleFactor = 1.0f;
    }
}