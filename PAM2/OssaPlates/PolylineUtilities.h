//
//  PolylineUtilities.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__PolylineUtilities__
#define __PAM2__PolylineUtilities__

#include <iostream>
#include <vector>
#include "Vec3f.h"
#include "Vec2f.h"

///returns reduced polyline with segments along the curve
std::vector<CGLA::Vec3f> reduceLineToEqualSegments(std::vector<CGLA::Vec3f> polylinePoints, float segmentLength);
std::vector<float> getCurvatures(std::vector<CGLA::Vec2f> polylinePoints);
#endif /* defined(__PAM2__PolylineUtilities__) */
