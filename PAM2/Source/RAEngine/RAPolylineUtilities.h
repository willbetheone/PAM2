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
    void getTangents3D(std::vector<CGLA::Vec3f>& tangents,
                     const std::vector<CGLA::Vec3f>& points);
    void getTangents2D(std::vector<CGLA::Vec2f>& tangents,
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
}

#endif /* defined(__PAM2__PolylineUtilities__) */
