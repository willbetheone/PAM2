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
    void reduceLineToEqualArcs(std::vector<CGLA::Vec3f>& reducedPoints,
                               const std::vector<CGLA::Vec3f>& polylinePoints,
                               float segmentLength);

    ///returns reduced polyline with equal linear segments along the curve
    void reduceLineToEqualSegments(std::vector<CGLA::Vec3f>& reducedPoints,
                                   const std::vector<CGLA::Vec3f>& polylinePoints,
                                   float segmentLength);
    
    ///calculate tangents fro given polyline
    void tangents(std::vector<CGLA::Vec3f>& tangents,
                  const std::vector<CGLA::Vec3f>& points);
    
    void centers(std::vector<CGLA::Vec3f>& centers,
                 const std::vector<CGLA::Vec3f>& points);


    ///check if 3 points are collinear, using corss product
    bool collinear(CGLA::Vec3f p1, CGLA::Vec3f p2, CGLA::Vec3f p3, double epsilone);
}

#endif /* defined(__PAM2__PolylineUtilities__) */
