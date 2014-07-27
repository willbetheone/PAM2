//
//  RAPolyLine.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAPolyLine__
#define __PAM2__RAPolyLine__

#include <iostream>
#include "RAMesh.h"

namespace RAEngine
{
    class RAPolyLine : public RAMesh
    {
    public:
        void setupShaders(const std::string vertexShader, const std::string fragmentShader);
        void bufferVertexDataToGPU(std::vector<CGLA::Vec3f>& points, CGLA::Vec4uc color, int lineMode);
        void bufferVertexDataToGPU(std::vector<CGLA::Vec3f>& points, CGLA::Vec4uc color, CGLA::Vec4uc color2, int lineMode);
        Bounds getBoundingBox() const override;
        void draw() const override;
    private:
        int lineMode;
    };
}
#endif /* defined(__PAM2__RAPolyLine__) */
