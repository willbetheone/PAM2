//
//  PAMManifold.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__PAMManifold__
#define __PAM2__PAMManifold__

#include <iostream>
#include "Manifold.h"
#include "Vec4uc.h"
#include "RAMesh.h"

namespace PAMMesh {
    
    class PAMManifold : public HMesh::Manifold, public RAEngine::RAMesh
    {
        
    public:
        
        PAMManifold();

        void setupShaders();
        
        void bufferVertexDataToGPU();
        
        void normalizeVertexCoordinates();
        
        void getVertexData(CGLA::Vec3f*& vertexPositions,
                           CGLA::Vec3f*& vertexNormals,
                           CGLA::Vec4uc*& vertexColors,
                           std::vector<unsigned int>*& indicies) const;
        
        void draw();
        
        int loadObjFile(const char* path) override;

        ///calculation intensive, not caching
        RAEngine::RABoundingBox getBoundingBox() override;
    };
}

#endif /* defined(__PAM2__PAMManifold__) */
