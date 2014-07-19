//
//  Plate.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-17.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__Plate__
#define __PAM2__Plate__

#include <iostream>
#include "RAMesh.h"
#include "Mat4x4f.h"

namespace Ossa
{
    class Plate : public RAEngine::RAMesh
    {
    public:
        void setSpline(const std::vector<CGLA::Vec3f>& points,
                       const std::vector<CGLA::Vec3f>& normals,
                       const std::vector<CGLA::Vec3f>& tangent);


        void setupShaders(const std::string& vertexShader,
                          const std::string& fragmentShader);
        RAEngine::Bounds getBoundingBox() const override;
        int  loadObjFile(const char* path) override;
        void draw() const;
        float scale = 1.0f;
    private:
        std::vector<CGLA::Mat4x4f> segmentMatricies;
        std::vector<CGLA::Mat4x4f> rotMatricies;
        std::vector<CGLA::Mat4x4f> translationMatricies;
    };
}

#endif /* defined(__PAM2__Plate__) */
