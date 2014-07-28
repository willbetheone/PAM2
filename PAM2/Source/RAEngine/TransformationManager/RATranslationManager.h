//
//  RATranslationManager.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-19.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RATranslationManager__
#define __PAM2__RATranslationManager__

#include <iostream>
#include "Mat4x4f.h"
#include "TouchScreenGestureStates.h"

namespace RAEngine {
    class RATranslationManager
    {
    public:
        RATranslationManager();
        CGLA::Mat4x4f getTranslationMatrix() const;
        CGLA::Vec3f getTranslationVector() const;
        void translate(CGLA::Vec3f translation);
        void handlePanGesture(GestureState state, CGLA::Vec3f translation);
        CGLA::Vec3f startPoint;
        void reset();
    private:
        CGLA::Mat4x4f translationMatrix;
        CGLA::Mat4x4f accumulatedTranslation;
        
    };
}

#endif /* defined(__PAM2__RATranslationManager__) */
