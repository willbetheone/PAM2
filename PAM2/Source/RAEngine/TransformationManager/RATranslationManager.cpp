//
//  RATranslationManager.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-19.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RATranslationManager.h"
#include "Vec3f.h"

namespace RAEngine {
    
    using namespace CGLA;
    
    RATranslationManager::RATranslationManager()
    {
        translationMatrix = identity_Mat4x4f();
        accumulatedTranslation = identity_Mat4x4f();
    }
    
    CGLA::Mat4x4f RATranslationManager::getTranslationMatrix() const
    {
        return translationMatrix;
    }
    
    void RATranslationManager::translate(CGLA::Vec3f translation)
    {
        translationMatrix =  translation_Mat4x4f(translation) * translationMatrix;
        accumulatedTranslation = translationMatrix;
    }
    
    void RATranslationManager::handlePanGesture(GestureState state, CGLA::Vec3f translation)
    {
//        UIPanGestureRecognizer* pan = (UIPanGestureRecognizer*)sender;
//        CGPoint point = [pan translationInView:pan.view];
//        
//        GLfloat ratio = pan.view.frame.size.height/pan.view.frame.size.width;
//        GLfloat x_ndc = point.x/pan.view.frame.size.width;
//        GLfloat y_ndc = -1*(point.y/pan.view.frame.size.height)*ratio;
        
        if (state == GestureState::Changed) {
//            Vec3f axis(x_ndc*2, y_ndc*2, 0);
            translationMatrix = translation_Mat4x4f(translation) * accumulatedTranslation;
        } else if (state == GestureState::Ended) {
            accumulatedTranslation = translationMatrix;
        }

    }

}