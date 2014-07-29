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
        void subdivide();
        void globalSmoothing();
        void showSkeleton(bool show);
        void clearVertexData();

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
        
#pragma mark - SMOOTHING
        void smoothAtPoint(CGLA::Vec3f touchPoint, float radius, int iter);

#pragma mark - BODY CREATION
        bool createBody(std::vector<CGLA::Vec3f>& polyline1,
                        std::vector<CGLA::Vec3f>& polyline2,
                        float zCoord,
                        std::vector<std::vector<CGLA::Vec3f>>& debugAllRibs,
                        bool debug);
        
#pragma mark - BRANCH CREATION
        void createBranch(std::vector<CGLA::Vec3f> touchPoints,
                          CGLA::Vec3f firstPoint,
                          bool touchedModel,
                          float touchSize,
                          float angularWidth);

#pragma mark - BUMP CREATION
        void startBumpCreation(CGLA::Vec3f touchPoint, float brushSize, float brushDepth);
        void continueBumpCreation(float brushDepth);
        void endBumpCreation();
        
#pragma mark - RIB SCALING
        void startScalingSingleRib(CGLA::Vec3f touchPoint,
                                   bool secondPointOnTheModel,
                                   float scale,
                                   float touchSize,
                                   bool anisotropic);
        void changeScalingSingleRib(float scale);
        void endScalingSingleRib(float scale);

#pragma mark - PIN POINT METHODS
        void createPinPoint(CGLA::Vec3f touchPoint, std::vector<CGLA::Vec3f>& pinPointRib);
        void deleteCurrentPinPoint();

#pragma mark - PIVOT POINT METHODS
        bool createPivotPoint(CGLA::Vec3f touchPoint);

#pragma mark - ROTATING THE BRANCH TREE
        void startBending(CGLA::Vec3f touchPoint, float angle);
        void continueBending(float angle);
        void endBendingWithAngle(float angle);

#pragma mark - SCALING THE BRANCH TREE
        void startScalingBranch(CGLA::Vec3f touchPoint, float scale);
        void continueScalingBranch(float scale);
        void endScalingBranchTree(float scale);

#pragma mark - TRANSLATION OF THE BRANCH TREE
        void startTranslatingBranchTree(CGLA::Vec3f touchPoint, CGLA::Vec3f translation);
        void continueTranslatingBranchTree(CGLA::Vec3f translation);
        void endTranslatingBranchTree(CGLA::Vec3f translation);

#pragma mark - POSING ROTATE
        void startPosingRotate(CGLA::Vec3f touchPoint, float angle);
        void continuePosingRotate(float angle);
        void endPosingRotate(float angle);
        
//#pragma mark - POSING_TRANSLATE
//        void statePosingTranslateWithTouchPoint(CGLA::Vec3f touchPoint, CGLA::Vec3f translation);
//        void continuePosingTranslate(CGLA::Vec3f translation);
//        void endPosingTranslate(CGLA::Vec3f translation);

#pragma mark - DELETE/DETACH BRANCH
        bool detachBranch(CGLA::Vec3f touchPoint);
        bool deleteBranch(CGLA::Vec3f touchPoint);
        bool moveDetachedBranch(CGLA::Vec3f touchPoint);
        bool attachDetachedBranch();

#pragma mark - ROTATE DETACHED BRANCH
        bool startRotateDetachedBranch(float angle);
        bool continueRotateDetachedBranch(float angle);
        void endRotateDetachedBranch(float angle);

#pragma mark - SCALE DETACHED BRANCH
        bool startScaleClonedBranch(float scale);
        void continueScaleClonedBranch(float scale);
        void endScaleClonedBranch(float scale);

    private:
        
        std::map<HMesh::VertexID, int> vertexIDtoIndex;
        Geometry::KDTree<CGLA::Vec3f, HMesh::VertexID>* kdTree;
        HMesh::HalfEdgeAttributeVector<EdgeInfo> edgeInfo;
        
        //COMMON
        HMesh::VertexID _pinVertexID;
        HMesh::HalfEdgeID _pinHalfEdgeID;
        HMesh::HalfEdgeID _pivotHalfEdgeID;
        std::set<HMesh::VertexID> _transformed_verticies;
        std::vector<HMesh::VertexID> _transition_verticies;
        
        //ROTATION
        float _rotAngle;
        HMesh::VertexAttributeVector<CGLA::Vec3f> _current_rot_position;
        CGLA::Vec3f _centerOfRotation;
        std::map<HMesh::VertexID, int> _vertexToLoop;
        std::vector<int> _loopsToDeform;
        std::map<int, float> _ringToDeformValue;
        HMesh::HalfEdgeID _deformDirHalfEdge;
        HMesh::HalfEdgeID _deformDirHalfEdgeEnd;
        
        //Translation
        CGLA::Vec3f _translationStart;
        CGLA::Vec3f _translationCurrent;
        
        //RIBS SCALING VARS
        float _scaleFactor;
        std::vector<HMesh::HalfEdgeID> _edges_to_scale;
        std::vector<HMesh::VertexID> _sculpt_verticies_to_scale;
        std::vector<CGLA::Vec3f> _centroids;
        std::vector<float> _scale_weight_vector;
        HMesh::VertexAttributeVector<CGLA::Vec3f> _current_scale_position;
        HMesh::VertexAttributeVector<CGLA::Vec3f> _anisotropic_projections;
        
        //BUMP CREATION VARS
        float _bumpBrushDepth;
        CGLA::Vec3d _bumpDirection;
        std::set<HMesh::VertexID> _bump_verticies;
        HMesh::VertexAttributeVector<int> _bump_current_displacement_vid_is_set;
        HMesh::VertexAttributeVector<float> _bump_verticies_weigths;
        HMesh::VertexAttributeVector<CGLA::Vec3f> _bump_current_displacement;
        HMesh::VertexAttributeVector<CGLA::Vec3f> _bump_current_norms;
        
        //DELETING AND REPOSITIONING OF THE BRANCH
        std::set<HMesh::VertexID> _detached_verticies;
        BOOL _deletingBranchFromBody;
        HMesh::HalfEdgeID _deleteBodyUpperRibEdge;
        HMesh::HalfEdgeID _deleteBranchLowerRibEdge;
        HMesh::HalfEdgeID _deleteDirectionSpineEdge;
        HMesh::HalfEdgeID _deleteBranchSecondRingEdge;
        int _deleteBranchNumberOfBoundaryRibs;
        HMesh::VertexID _newAttachVertexID;
        Vec _zRotateVec;
        Vec _zRotatePos;
        
        //CLONING
        HMesh::HalfEdgeID _cloningDirection;
        HMesh::HalfEdgeID _cloningBodyUpperRibEdge;
        HMesh::HalfEdgeID _cloningBranchLowerRibEdge;
        HMesh::HalfEdgeID _cloningSecondRing;
        HMesh::VertexID _newClonedVertexID;
        int _copyNumBoundaryRibSegments;
        
        
        void bufferVertexDataToGPU();
        
        void traceEdgeInfo();
        
        void getVertexData(CGLA::Vec3f*& vertexPositions,
                           CGLA::Vec3f*& vertexNormals,
                           CGLA::Vec4uc*& vertexColors,
                           std::vector<unsigned int>*& indicies,
                           CGLA::Vec4uc*& wireframeColor,
                           std::vector<unsigned int>*& wireframeIndicies);
        
#pragma mark - BODY CREATION UTILITIES
        void populateManifold(std::vector<std::vector<CGLA::Vec3f>>& allRibs);
        
        int indexForCentroid(int centeroid, int rib, int totalCentroid, int totalRib);

#pragma mark - BRANCH CREATION UTILITIES
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

#pragma mark - PIVOT POINT UTILITIES
        bool toPivotFromPinDirection(HMesh::HalfEdgeID& toPivothID);
        bool setTransformedArea();
        bool setTransformedAreaForPosingRotate(HMesh::Walker& awayFromTransform);
        
#pragma mark - UTILITIES
        bool closestVertexID_3D(const CGLA::Vec3f& point, HMesh::VertexID& vid);
        
        bool closestVertexID_2D(const CGLA::Vec3f& point, HMesh::VertexID& vid);
        
        std::set<HMesh::VertexID> allVerticiesInDirectionHID(HMesh::HalfEdgeID hID);
        
        std::set<HMesh::VertexID> allVerticiesInDirection(HMesh::Walker deleteDir);

#pragma mark - ROTATING THE BRANCH TREE UTILITIES
        void rotateRingsFrom(HMesh::HalfEdgeID pivotDirHID, HMesh::HalfEdgeID pivotHalfEdge);
        
#pragma mark - POSING UTILITIES
        HMesh::Walker biggerSide(HMesh::HalfEdgeID ribLoop, std::set<HMesh::VertexID>& returnSet);

#pragma mark - SMOOTHING
        void neighbours(std::set<HMesh::VertexID>& neighbours, std::vector<HMesh::VertexID>& verticies, float brush_size);
        
        void neighbours(std::vector<HMesh::VertexID>& neighbours, HMesh::VertexID vID, std::vector<float>& weights, float brush_size);
        
        void smoothPole(HMesh::VertexID vID, int depth, int iter);
        
        std::set<HMesh::VertexID> smoothAlongRib(HMesh::HalfEdgeID rib, int iter, bool isSpine, float brushSize);
        
        std::set<HMesh::VertexID> smoothVerticies(std::vector<HMesh::VertexID> vIDs, int iter, bool isSpine, float brushSize);

#pragma mark - DELETE/DETACH BRANCH UTILITIES
        void closeHole(HMesh::HalfEdgeID hID, int& numOfEdges);
        
#pragma mark -  UPDATE GPU DATA
        void updateVertexPositionOnGPU_Vector(std::vector<HMesh::VertexID>& verticies);
        void updateVertexNormOnGPU_Vector(std::vector<HMesh::VertexID>& verticies);
        void updateVertexPositionOnGPU_Set(std::set<HMesh::VertexID>& verticies);
        void updateVertexNormOnGPU_Set(std::set<HMesh::VertexID>& verticies);
        
        void changeVerticiesColor_Vector(std::vector<HMesh::VertexID>& vertecies, CGLA::Vec4uc selectColor);
        void changeVerticiesColor_Set(std::set<HMesh::VertexID>& vertecies, CGLA::Vec4uc selectColor);
        void changeVerticiesColor_Vector(std::vector<HMesh::VertexID>& vertecies, bool isSelected);
        void changeVerticiesColor_Set(std::set<HMesh::VertexID>& vertecies, bool isSelected);
        
#pragma mark - DELETE
        float signedAngleBetweenReferenceVector3(CGLA::Vec3f refVector, CGLA::Vec3f vector);
    };
}



#endif /* defined(__PAM2__PAMManifold__) */
