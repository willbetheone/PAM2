//
//  PolylineUtilities.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "PolylineUtilities.h"
#include "RALogManager.h"

using namespace CGLA;
using namespace std;

std::vector<CGLA::Vec3f> reduceLineToEqualSegments(std::vector<CGLA::Vec3f> polylinePoints, float segmentLength)
{
    //Add first centroid for pole
    vector<Vec3f> centroids;
    float accumLen = 0.0f;
    Vec3f lastCentroid = polylinePoints[0];
    centroids.push_back(lastCentroid);
    
    RA_LOG_VERY_VERBOSE("Drawing new polyline");
    //Add all other centroids
    int i = 1;
    while (i < polylinePoints.size()) {
        Vec3f centroid = polylinePoints[i];
        float curLen = (centroid - lastCentroid).length();
        if (!curLen) {
            RA_LOG_ERROR("Incorrect length between points on a polyline %f", curLen);
        }
        accumLen += curLen;
        
        if (accumLen >= segmentLength) {
            Vec3f dir = normalize(centroid - lastCentroid);
            Vec3f newCenter = lastCentroid + (segmentLength - (accumLen - curLen)) * dir;
            centroids.push_back(newCenter);
            accumLen = 0.0f;
            lastCentroid = newCenter;
        } else {
            lastCentroid = centroid;
            i++;
        }
    }
    
    return centroids;
}







