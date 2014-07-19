//
//  RAPlateSegment.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-14.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__PlateSegment__
#define __PAM2__PlateSegment__

#include <iostream>
#include "RAMesh.h"

namespace Ossa
{
    class PlateSegment : public RAEngine::RAMesh
    {
    public:
        PlateSegment(const CGLA::Vec3f&  center, const CGLA::Vec3f& norm);        
        void setupShaders(const std::string vertexShader, const std::string fragmentShader);
        void resetTranslation();
        int  loadObjFile(const char* path) override;
        RAEngine::Bounds getBoundingBox() const override;
        void draw() const;
        void setCenterAndNormal(const CGLA::Vec3f& center, const CGLA::Vec3f& norm);
    private:
    };
}


#endif /* defined(__PAM2__PlateSegment__) */
