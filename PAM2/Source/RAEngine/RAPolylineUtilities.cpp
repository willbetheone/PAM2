//
//  PolylineUtilities.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAPolylineUtilities.h"
#include "RALogManager.h"

namespace RAEngine {
    
    using namespace CGLA;
    using namespace std;

    void reduceLineToEqualArcs(std::vector<CGLA::Vec3f>& reducedPoints,
                               const std::vector<CGLA::Vec3f>& polylinePoints,
                               float segmentLength)
    {
        typedef CGLA::Vec3f Vec;
        typedef float T;
        //Add first centroid for pole

        T accumLen = 0.0f;
        Vec lastCentroid = polylinePoints[0];
        reducedPoints.push_back(lastCentroid);
        
        RA_LOG_VERY_VERBOSE("Drawing new polyline");
        //Add all other centroids
        int i = 1;
        while (i < polylinePoints.size()) {
            Vec centroid = polylinePoints[i];
            T curLen = (centroid - lastCentroid).length();
            if (!curLen) {
                RA_LOG_ERROR("Incorrect length between points on a polyline %f", curLen);
            }
            accumLen += curLen;
            
            if (accumLen >= segmentLength) {
                Vec dir = normalize(centroid - lastCentroid);
                Vec newCenter = lastCentroid + (segmentLength - (accumLen - curLen)) * dir;
                reducedPoints.push_back(newCenter);
                accumLen = 0.0f;
                lastCentroid = newCenter;
            } else {
                lastCentroid = centroid;
                i++;
            }
        }
    }
    
    
    void reduceLineToEqualSegments(std::vector<CGLA::Vec3f>& reducedPoints,
                                   const std::vector<CGLA::Vec3f>& polylinePoints,
                                   float segmentLength)
    {
        typedef CGLA::Vec3f Vec;
        typedef float T;

        Vec A_point = polylinePoints[0]; // last centroid added to the reduced line
        Vec B_point = polylinePoints[0]; // previous point at a distance less than segmentLen from A.
        reducedPoints.push_back(A_point);
        
        RA_LOG_VERY_VERBOSE("Drawing new polyline");

        //Add all other centroids
        int i = 1;
        while (i < polylinePoints.size())
        {
            Vec C_point = polylinePoints[i]; // current point along given line
            Vec AC_vec = C_point - A_point;
            T AC_len = AC_vec.length();
            float difference = fabs(AC_len - segmentLength);
            
            if (difference < 1e-3) {
                RA_LOG_INFO("Small difference %i", i);
                Vec R_point = C_point;
                reducedPoints.push_back(R_point);
                A_point = R_point;
                B_point = R_point;
                i++;
            }
            else if (AC_len > segmentLength) {
    //            Vec B_point = polylinePoints[i-1]; // previous point at a distance less than segmentLen from A.
                Vec AB_vec = B_point - A_point;
                T AB_len = AB_vec.length();
                Vec CB_vec = C_point - B_point;
                T CB_len = CB_vec.length();

                //Next point to be added to the reduced set  (name it R) is between B and C. So find it.
                //Not AR_len == segmentLength

                if (collinear(A_point, B_point, C_point, 1e-5)) { //special case. points are collinear
                    RA_LOG_INFO("Collinear %i", i);
                    Vec R_point = segmentLength * normalize(AC_vec) + A_point;
                    reducedPoints.push_back(R_point);
                    A_point = R_point;
                    B_point = R_point;
                } else {
                    T ABC_angle = acos((pow(AB_len, 2) + pow(CB_len, 2) - pow(AC_len, 2))/(2*AB_len*CB_len));
                    assert(ABC_angle > 0);
                    T AR_len = segmentLength;
                    T BRA_angle = asin(AB_len * sin(ABC_angle) / AR_len);
                    assert(BRA_angle > 0);
                    T BAR_angle = M_PI - ABC_angle - BRA_angle;
                    assert(BAR_angle > 0);
                    T BR_len = sqrtf(pow(AB_len, 2) + pow(AR_len, 2) - 2*AB_len*AR_len*cos(BAR_angle));
                    assert(BR_len > 0);
                    Vec R_point = BR_len * normalize(CB_vec) + B_point;
                    reducedPoints.push_back(R_point);
                    A_point = R_point;
                    B_point = R_point;
                }
            }
             else if (AC_len >= segmentLength) {
                Vec R_point = C_point;
                reducedPoints.push_back(R_point);
                A_point = R_point;
                B_point = R_point;
                i++;
            } else {
                B_point = C_point;
                i++;
            }
        }
    }
    
    void tangents(std::vector<CGLA::Vec3f>& tangents,
                  const std::vector<CGLA::Vec3f>& points)
    {
        for (int i = 0; i < points.size() - 1; i++) {
            Vec3f t = points[i+1] - points[i];
            tangents.push_back(normalize(t));
        }
    }
    
    void centers(std::vector<CGLA::Vec3f>& centers,
                 const std::vector<CGLA::Vec3f>& points)
    {
        for (int i = 0; i < points.size() - 1; i++) {
            Vec3f c = 0.5f * (points[i+1] + points[i]);
            centers.push_back(c);
        }
    }


    bool collinear(CGLA::Vec3f p1, CGLA::Vec3f p2, CGLA::Vec3f p3, double epsilone)
    {
        double cros = 0.5 * cross(p2 - p1, p3 - p1).length();
        return cros <= epsilone;
    }

}


