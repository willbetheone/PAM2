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
#include "AttributeVector.h"
#include <map>
#include <set>

class EdgeInfo;

namespace PAMMesh
{
    class PAMManifold : public HMesh::Manifold, public RAEngine::RAMesh
    {
    public:
        PAMManifold();
        ~PAMManifold();

        void setupShaders();
        
        void updateMesh();
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
        
        enum class Modification
        {
            NONE,
            SCULPTING_SCALING,
            SCULPTING_ANISOTROPIC_SCALING,
            SCULPTING_BUMP_CREATION,
            
            PIN_POINT_SET,
            
            BRANCH_ROTATION,
            BRANCH_SCALING,
            BRANCH_TRANSLATION,
            
            BRANCH_DETACHED,
            BRANCH_DETACHED_AN_MOVED,
            BRANCH_DETACHED_ROTATE,
            
            BRANCH_COPIED_BRANCH_FOR_CLONING,
            BRANCH_COPIED_AND_MOVED_THE_CLONE,
            BRANCH_CLONE_ROTATION,
            BRANCH_CLONE_SCALING,
            
            BRANCH_POSE_ROTATE,
            BRANCH_POSE_TRANSLATE
        };
        Modification modState;

        
#pragma mark - BODY CREATION
        bool createBody(std::vector<CGLA::Vec3f>& polyline1,
                        std::vector<CGLA::Vec3f>& polyline2,
                        float zCoord,
                        std::vector<std::vector<CGLA::Vec3f>>& debugAllRibs,
                        bool debug);
        
#pragma mark - BRANCH CREATION
        void endCreateBranchBended(std::vector<CGLA::Vec3f> touchPoints,
                                   CGLA::Vec3f firstPoint,
                                   bool touchedModel,
                                   float touchSize);

#pragma mark - BUMP CREATION
        
#pragma mark - RIB SCALING
        void startScalingSingleRib(CGLA::Vec3f touchPoint,
                                   bool secondPointOnTheModel,
                                   float scale,
                                   float velocity,
                                   float touchSize,
                                   bool anisotropic);
        void changeScalingSingleRib(float scale);
        void endScalingSingleRib(float scale);

    private:
        
        std::map<HMesh::VertexID, int> vertexIDtoIndex;
        Geometry::KDTree<CGLA::Vec3f, HMesh::VertexID>* kdTree;
        HMesh::HalfEdgeAttributeVector<EdgeInfo> edgeInfo;
        
        std::vector<HMesh::HalfEdgeID> _edges_to_scale;
        std::vector<HMesh::VertexID> _sculpt_verticies_to_scale;
        HMesh::VertexAttributeVector<CGLA::Vec3f> _current_scale_position;
        std::vector<CGLA::Vec3f> _centroids;
        std::vector<float> _scale_weight_vector;
        HMesh::VertexAttributeVector<CGLA::Vec3f> _anisotropic_projections;
        
        float _scaleFactor;
        
        void bufferVertexDataToGPU();
        
        void traceEdgeInfo();
        
        void getVertexData(CGLA::Vec3f*& vertexPositions,
                           CGLA::Vec3f*& vertexNormals,
                           CGLA::Vec4uc*& vertexColors,
                           std::vector<unsigned int>*& indicies,
                           CGLA::Vec4uc*& wireframeColor,
                           std::vector<unsigned int>*& wireframeIndicies);
        
#pragma mark - BODY CREATION
        void populateManifold(std::vector<std::vector<CGLA::Vec3f>>& allRibs);
        
        int indexForCentroid(int centeroid, int rib, int totalCentroid, int totalRib);

#pragma mark - BRANCH CREATION
        int branchWidthForAngle(float angle, HMesh::VertexID vID);
        
        bool createHoleAtVertex(HMesh::VertexID vID,int width,HMesh::VertexID& newPoleID,float& bWidth,
                                CGLA::Vec3f& holeCenter,CGLA::Vec3f& holeNorm,HMesh::HalfEdgeID& boundayHalfEdge);
        
        bool createBranchAtVertex(HMesh::VertexID vID,int numOfSegments,HMesh::VertexID& newPoleID,float& bWidth);
        
        void populateNewLimb(std::vector<std::vector<CGLA::Vec3f>>& allRibs,std::vector<CGLA::Vec3f>& vertices,
                             std::vector<int>& faces,std::vector<int>& indices);
        
        void allVerticiesAndHalfEdges(std::vector<HMesh::VertexID>& verticies,
                                      std::vector<HMesh::HalfEdgeID>& halfedges,HMesh::VertexID vID);
        
        bool boundaryHalfEdgeForClonedMesh(HMesh::HalfEdgeID& boundaryHalfedge,
                                           std::vector<HMesh::HalfEdgeID>& newHalfEdges);
        
        void stitchBranchToBody(HMesh::HalfEdgeID branchHID,HMesh::HalfEdgeID bodyHID);
        
        int limbIndexForCentroid(int centeroid,int rib,int totalCentroid, int totalRib);
        
#pragma mark - UTILITIES
        bool closestVertexID_3D(const CGLA::Vec3f& point, HMesh::VertexID& vid);
        
        bool closestVertexID_2D(const CGLA::Vec3f& point, HMesh::VertexID& vid);
        
#pragma mark - SMOOTHING
        void neighbours(std::set<HMesh::VertexID>& neighbours, std::vector<HMesh::VertexID>& verticies, float brush_size);
        
        void neighbours(std::vector<HMesh::VertexID>& neighbours, HMesh::VertexID vID, std::vector<float>& weights, float brush_size);
        
        void smoothPole(HMesh::VertexID vID, int depth, int iter);
        
        std::set<HMesh::VertexID> smoothAlongRib(HMesh::HalfEdgeID rib, int iter, bool isSpine, float brushSize);
        
        std::set<HMesh::VertexID> smoothVerticies(std::vector<HMesh::VertexID> vIDs, int iter, bool isSpine, float brushSize);
        
#pragma mark -  UPDATE GPU DATA
        void updateVertexPositionOnGPU_Vector(std::vector<HMesh::VertexID>& verticies);
        
        void updateVertexNormOnGPU_Vector(std::vector<HMesh::VertexID>& verticies);
        
        void updateVertexPositionOnGPU_Set(std::set<HMesh::VertexID>& verticies);
        
        void updateVertexNormOnGPU_Set(std::set<HMesh::VertexID>& verticies);
    };
}



#endif /* defined(__PAM2__PAMManifold__) */
