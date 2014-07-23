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
#include "KDTree.h"

namespace PAMMesh
{
    class PAMManifold : public HMesh::Manifold, public RAEngine::RAMesh
    {
    public:
        PAMManifold();
        ~PAMManifold();

        void setupShaders();
        
        void bufferVertexDataToGPU();
        
        void normalizeVertexCoordinates();
        
        void draw() const override;
        void drawToDepthBuffer();
        
        int loadObjFile(const char* path) override;
        int loadPAMObjFile(const char* path);

        ///calculation intensive, not caching
        RAEngine::Bounds getBoundingBox() const override;
        
        ///get normal of the closest vertex to point
        bool normal(const CGLA::Vec3f& point, CGLA::Vec3f& norm);
        
        ///add kd tree support
        void buildKDTree();
        
        /* MODELING FUNCTIONS */
        void createBody(std::vector<CGLA::Vec3f>& polyline1,
                        std::vector<CGLA::Vec3f>& polyline2);
        
    private:
        Geometry::KDTree<CGLA::Vec3f, HMesh::VertexID>* kdTree = nullptr;
        
        bool closestVertexID_3D(const CGLA::Vec3f& point, HMesh::VertexID& vid);

        void getVertexData(CGLA::Vec3f*& vertexPositions,
                           CGLA::Vec3f*& vertexNormals,
                           CGLA::Vec4uc*& vertexColors,
                           std::vector<unsigned int>*& indicies) const;
        void populateManifold(std::vector<std::vector<CGLA::Vec3f>>& allRibs);
        int indexForCentroid(int centeroid, int rib, int totalCentroid, int totalRib);
    };
}



#endif /* defined(__PAM2__PAMManifold__) */
