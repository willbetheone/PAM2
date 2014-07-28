//
//  PolylineUtilities.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAPolylineUtilities__
#define __PAM2__RAPolylineUtilities__

#include <iostream>
#include <vector>
#include "Vec3f.h"
#include "Vec2f.h"
#include "ArithVecFloat.h"
namespace RAEngine {
    
    ///returns reduced polyline with equal arcs along the curve
    void reduceLineToEqualArcs3D(std::vector<CGLA::Vec3f>& reducedPoints,
                               const std::vector<CGLA::Vec3f>& polylinePoints,
                               float segmentLength);

    void reduceLineToEqualArcs2D(std::vector<CGLA::Vec2f>& reducedPoints,
                               const std::vector<CGLA::Vec2f>& polylinePoints,
                               float segmentLength);


    ///returns reduced polyline with equal linear segments along the curve
    void reduceLineToEqualSegments3D(std::vector<CGLA::Vec3f>& reducedPoints,
                                   const std::vector<CGLA::Vec3f>& polylinePoints,
                                   float segmentLength);

    void reduceLineToEqualSegments2D(std::vector<CGLA::Vec2f>& reducedPoints,
                                   const std::vector<CGLA::Vec2f>& polylinePoints,
                                   float segmentLength);

    ///calculate tangents fro given polyline
    template<class T, class V, unsigned int N>
    void getTangents3D(std::vector<CGLA::ArithVecFloat<T,V,N>>& tangents,
                       const std::vector<CGLA::ArithVecFloat<T,V,N>>& points);

    void getTangents2D(std::vector<CGLA::Vec2f>& tangents,
                       const std::vector<CGLA::Vec2f>& points);
    
    void getTangentsAndNormals2D(std::vector<CGLA::Vec2f>& tangents,
                                 std::vector<CGLA::Vec2f>& normals,
                                 const std::vector<CGLA::Vec2f>& points);

    
    ///calculate tangents and normals for a given polyline
    void getNormals2D(std::vector<CGLA::Vec2f>& normals,
                    const std::vector<CGLA::Vec2f>& tangents,
                    const std::vector<CGLA::Vec2f>& skeleton);

    ///calculate centers
    void getCenters3D(std::vector<CGLA::Vec3f>& centers,
                    const std::vector<CGLA::Vec3f>& points);

    ///check if 3 points are collinear, using corss product
    bool collinear3D(CGLA::Vec3f p1, CGLA::Vec3f p2, CGLA::Vec3f p3, double epsilone);
    bool collinear2D(CGLA::Vec2f p1, CGLA::Vec2f p2, CGLA::Vec2f p3, double epsilone);
    
    bool lineSegmentRayIntersection(CGLA::Vec2f p1, CGLA::Vec2f p2, CGLA::Vec2f q, CGLA::Vec2f s, float& rU);
    
    
    bool getSmoothCurve(const std::vector<CGLA::Vec2f>& input,
                        std::vector<CGLA::Vec2f>* output,
                        std::vector<CGLA::Vec2f>* tangents,
                        std::vector<CGLA::Vec2f>* normals,
                        float t);

    void laplacianSmoothing(std::vector<CGLA::Vec2f> input,
                            std::vector<CGLA::Vec2f>& output,
                            int iter,
                            float d);

    void laplacianSmoothing(std::vector<CGLA::Vec3f> input,
                            std::vector<CGLA::Vec3f>& output,
                            int iter,
                            float d);
    
    void normals3D(std::vector<CGLA::Vec3f>& normals,
                   std::vector<CGLA::Vec3f>& tangents,
                   std::vector<CGLA::Vec3f>& skeleton);
    
    CGLA::Vec3f orthogonalVectorTo(CGLA::Vec3f vector);
    
}

#endif /* defined(__PAM2__PolylineUtilities__) */
