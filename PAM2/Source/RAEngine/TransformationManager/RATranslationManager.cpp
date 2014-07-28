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
    
    CGLA::Vec3f RATranslationManager::getTranslationVector() const
    {
        Vec3f translation = Vec3f(translationMatrix[0][3], translationMatrix[1][3], translationMatrix[2][3]);
        return translation;
    }
    
    void RATranslationManager::translate(CGLA::Vec3f translation)
    {
        translationMatrix =  translation_Mat4x4f(translation) * translationMatrix;
        accumulatedTranslation = translationMatrix;
    }
    
    void RATranslationManager::handlePanGesture(GestureState state, CGLA::Vec3f translation)
    {
        if (state == GestureState::Began) {
            startPoint = translation;
        } else if (state == GestureState::Changed) {            
            translationMatrix = translation_Mat4x4f(translation - startPoint) * accumulatedTranslation;
        } else if (state == GestureState::Ended) {
            accumulatedTranslation = translationMatrix;
        }

    }
    
    void RATranslationManager::reset()
    {
        translationMatrix = identity_Mat4x4f();
        accumulatedTranslation = identity_Mat4x4f();
    }

}