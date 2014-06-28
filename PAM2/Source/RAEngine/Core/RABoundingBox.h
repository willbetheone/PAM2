//
//  RABoundingBox.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-23.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RABoundingBox__
#define __PAM2__RABoundingBox__

#include <iostream>
#include "Vec3f.h"
#include "Vec4uc.h"
#include "RAMesh.h"

namespace RAEngine {

    class RABoundingBox : public RAMesh
    {
    public:
        RABoundingBox();
        RABoundingBox(CGLA::Vec3f pmin, CGLA::Vec3f pmax);
        
        void bufferVertexDataToGPU();
        void setupShaders(const std::string vertexShader, const std::string fragmentShader);

        void draw() const override;
        Bounds getBoundingBox() const override;
        
    private:
        
        void getVertexData(CGLA::Vec3f*& vertexPositions,
                           CGLA::Vec4uc*& vertexColors,
                           std::vector<unsigned int>*& indicies);
        
        CGLA::Vec3f minBound;
        CGLA::Vec3f maxBound;
        CGLA::Vec3f center;
        
        float radius;
        float width;
        float height;
        float depth;
    };
}

#endif /* defined(__PAM2__RABoundingBox__) */
