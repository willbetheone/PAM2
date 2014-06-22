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

namespace RAEngine {
    class RATranslationManager
    {
    public:
        RATranslationManager();
        CGLA::Mat4x4f getTranslationMatrix() const;
        void handlePanGesture(UIGestureRecognizer* sender);
    private:
        CGLA::Mat4x4f translationMatrix;
        CGLA::Mat4x4f accumulatedTranslation;
    };
}

#endif /* defined(__PAM2__RATranslationManager__) */
