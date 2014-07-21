//
//  PAMSettingsManager.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-21.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "PAMSettingsManager.h"

namespace PAMMesh
{
    PAMSettingsManager::PAMSettingsManager()
    {
        transform = false;
        showSkeleton = false;
        smoothingBrushSize = 1.0;
        baseSmoothingIterations = 15;
        thinBranchWidth = 20;
        mediumBranchWidth = 40;
        largeBranchWidth = 80;
        spineSmoothing = true;
        poleSmoothing = true;
        sculptScalingType = ScultpScalingType::Silhouette;
        silhouetteScalingBrushSize = 90;
        tapSmoothing = 1;
    }
    
    PAMSettingsManager& PAMSettingsManager::getInstance()
    {
        static PAMSettingsManager instance;
        return instance;
    }
}