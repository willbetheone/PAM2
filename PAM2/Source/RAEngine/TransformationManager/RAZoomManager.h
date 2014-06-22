//
//  RAZoomManager.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-19.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAZoomManager__
#define __PAM2__RAZoomManager__

#include <iostream>

namespace RAEngine {
    class RAZoomManager
    {
    public:
        float getScaleFactor() const;
        void handlePinchGesture(UIGestureRecognizer* sender);
    private:
        float scaleFactor = 1.0f;
        float curFactor;
    };
}

#endif /* defined(__PAM2__RAZoomManager__) */
