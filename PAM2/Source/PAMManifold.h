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
        // Default constructor
        PAMManifold() : HMesh::Manifold() {};

        void normalizeVertexCoordinates() noexcept;
        
        void getVertexData(CGLA::Vec3f*& vertexPositions,
                           CGLA::Vec3f*& vertexNormals,
                           CGLA::Vec4uc*& vertexColors,
                           std::vector<size_t>*& indicies) const noexcept;
    };
}

#endif /* defined(__PAM2__PAMManifold__) */
