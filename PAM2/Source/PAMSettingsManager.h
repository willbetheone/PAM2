//
//  PAMSettingsManager.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-21.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__PAMSettingsManager__
#define __PAM2__PAMSettingsManager__

#include <iostream>

namespace PAMMesh
{
    class PAMSettingsManager
    {
    public:
        enum class ScultpScalingType {Circular, Silhouette};

        static PAMSettingsManager& getInstance();
        
        bool transform;
        bool showSkeleton;
        float smoothingBrushSize;
        float branchWidth;
        int baseSmoothingIterations;
        bool spineSmoothing;
        bool poleSmoothing;
        ScultpScalingType sculptScalingType;
        float silhouetteScalingBrushSize;
        float tapSmoothing;
        
    private:
        PAMSettingsManager();
        PAMSettingsManager(PAMSettingsManager const&);
        void operator=(PAMSettingsManager const&);
    };
}

#endif /* defined(__PAM2__PAMSettingsManager__) */
