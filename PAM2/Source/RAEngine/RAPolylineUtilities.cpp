//
//  PolylineUtilities.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAPolylineUtilities.h"
#include "RALogManager.h"
#include "Wm5IntrRay2Segment2.h"
#include "Wm5BSplineCurveFit.h"
#include "Wm5BSplineCurve2.h"
#include <algorithm>
#include "Quatf.h"

using namespace Wm5;

namespace RAEngine {
    
    using namespace CGLA;
    using namespace std;

    void reduceLineToEqualArcs3D(std::vector<CGLA::Vec3f>& reducedPoints,
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
    
    void reduceLineToEqualArcs2D(std::vector<CGLA::Vec2f>& reducedPoints,
                               const std::vector<CGLA::Vec2f>& polylinePoints,
                               float segmentLength)
    {
        typedef CGLA::Vec2f Vec;
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

    
    
    void reduceLineToEqualSegments3D(std::vector<CGLA::Vec3f>& reducedPoints,
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

                if (collinear3D(A_point, B_point, C_point, 1e-5)) { //special case. points are collinear
//                    RA_LOG_INFO("Collinear %i", i);
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
    
    void reduceLineToEqualSegments2D(std::vector<CGLA::Vec2f>& reducedPoints,
                                     const std::vector<CGLA::Vec2f>& polylinePoints,
                                     float segmentLength)
    {
        typedef CGLA::Vec2f Vec;
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
                
                if (collinear2D(A_point, B_point, C_point, 1e-5)) { //special case. points are collinear
//                    RA_LOG_INFO("Collinear %i", i);
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

    
    template<class T, class V, unsigned int N>
    void getTangents3D(std::vector<CGLA::ArithVecFloat<T,V,N>>& tangents,
                       const std::vector<CGLA::ArithVecFloat<T,V,N>>& points)
    {
        assert(points.size() > 1);
        for (int i = 0; i < points.size(); i++) {
            CGLA::ArithVecFloat<T,V,N> t;
            if (i == 0) {
                t = points[1] - points[0];
            } else if (i == points.size() - 1) {
                t = points[i] - points[i-1];
            } else {
                t = points[i+1] - points[i-1];
            }
            tangents.push_back(normalize(t));
        }
    }
    
    void getTangents2D(std::vector<CGLA::Vec2f>& tangents,
                       const std::vector<CGLA::Vec2f>& points)
    {
        assert(points.size() > 1);
        for (int i = 0; i < points.size(); i++) {
            Vec2f t;
            if (i == 0) {
                t = points[1] - points[0];
            } else if (i == points.size() - 1) {
                t = points[i] - points[i-1];
            } else {
                t = points[i+1] - points[i-1];
            }
            tangents.push_back(normalize(t));
        }
    }

    void getNormals2D(std::vector<CGLA::Vec2f>& normals,
                      const std::vector<CGLA::Vec2f>& tangents,
                      const std::vector<CGLA::Vec2f>& skeleton)
    {
        //Get norm vectors
        for (int i = 0; i < skeleton.size(); i++) {
            Vec2f tan = tangents[i];
            Vec2f norm = Vec2f(tan[1], -1*tan[0]);
            normals.push_back(norm);
        }
    }
    
    void getTangentsAndNormals2D(std::vector<CGLA::Vec2f>& tangents,
                                 std::vector<CGLA::Vec2f>& normals,
                                 const std::vector<CGLA::Vec2f>& points)
    {
        assert(points.size() > 1);
        for (int i = 0; i < points.size(); i++) {
            Vec2f t;
            if (i == 0) {
                t = points[1] - points[0];
            } else if (i == points.size() - 1) {
                t = points[i] - points[i-1];
            } else {
                t = points[i+1] - points[i-1];
            }
            Vec2f n = Vec2f(t[1], -1*t[0]);
            tangents.push_back(normalize(t));
            normals.push_back(normalize(n));
        }
    }

    
    void getCenters3D(std::vector<CGLA::Vec3f>& centers,
                    const std::vector<CGLA::Vec3f>& points)
    {
        for (int i = 0; i < points.size() - 1; i++) {
            Vec3f c = 0.5f * (points[i+1] + points[i]);
            centers.push_back(c);
        }
    }

    bool collinear3D(CGLA::Vec3f p1, CGLA::Vec3f p2, CGLA::Vec3f p3, double epsilone)
    {
        double cros = 0.5 * cross(p2 - p1, p3 - p1).length();
        return cros <= epsilone;
    }
    
    bool collinear2D(CGLA::Vec2f p1, CGLA::Vec2f p2, CGLA::Vec2f p3, double epsilone)
    {
        float cros = abs(p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]))/2;
        return cros <= epsilone;
    }
    
    bool lineSegmentRayIntersection(CGLA::Vec2f p1, CGLA::Vec2f p2, CGLA::Vec2f q, CGLA::Vec2f s, float& rU)
    {
        Ray2f ray =  Ray2f(Vector2f(q[0], q[1]), Vector2f(s[0], s[1]));
        Segment2f seg = Segment2f(Vector2f(p1[0], p1[1]), Vector2f(p2[0], p2[1]));
        
        IntrRay2Segment2f intr = IntrRay2Segment2f(ray, seg);
        
        if (intr.Find()) {
            if (intr.GetQuantity() == 1) {
                Vector2f intrPoint = intr.GetPoint(0);
                Vector2f qWm5 = Vector2f(q[0], q[1]);
                rU = (intrPoint - qWm5).Length();
                return true;
            }
        }
        return false;
    }
    
    bool getSmoothCurve(const std::vector<CGLA::Vec2f>& input,
                        std::vector<CGLA::Vec2f>* output,
                        std::vector<CGLA::Vec2f>* tangents,
                        std::vector<CGLA::Vec2f>* normals,
                        float t)
    {
        assert(output != nullptr);
        assert(input.size() >= 5);
        assert(t > 0);
        
        if (input.size() < 5) {
            RA_LOG_ERROR("Input has less than 4 points");
            return false;
        }
        
        int degree = 3;
        int numOfControls = std::max((int)(input.size()/2), 5) ;
//        int numOfControls = input.size();
        
        BSplineCurveFitf fit = BSplineCurveFitf(2, input.size(), (const float*)input.data(), degree, numOfControls);
        BSplineCurve2f curve = BSplineCurve2f(fit.GetControlQuantity(), (const Vector2f*)fit.GetControlData(), degree, false, true);
        float length = curve.GetLength(curve.GetMinTime(), curve.GetMaxTime());
        
        for (int i = 0; i * t <= length; ++i)
        {
            float time  = curve.GetTime(i * t);
            Vector2f pos, tan, norm;
            curve.GetFrame(time, pos, tan, norm);
            
            output->push_back(Vec2f(pos[0], pos[1]));

            if (tangents) {
                tangents->push_back(Vec2f(tan[0], tan[1]));
            }
            if (normals) {
                normals->push_back(Vec2f(norm[0], norm[1]));
            }            
        }
        
        return true;
    }
    
    void laplacianSmoothing(std::vector<CGLA::Vec2f> input,
                            std::vector<CGLA::Vec2f>& output,
                            int iter,
                            float d)
    {
        output = input;
        for (int j = 0; j < iter; j++) {
            for (int i = 1; i < input.size() - 1; i++) {
                Vec2f lap = 0.5f * (input[i-1] + input[i+1]) - input[i];
                output[i] = input[i] + d*lap;
            }
            input = output;
        }
    }
    
    void laplacianSmoothing(std::vector<CGLA::Vec3f> input,
                            std::vector<CGLA::Vec3f>& output,
                            int iter,
                            float d)
    {
        output = input;
        for (int j = 0; j < iter; j++) {
            for (int i = 1; i < input.size() - 1; i++) {
                Vec3f lap = 0.5f * (input[i-1] + input[i+1]) - input[i];
                output[i] = input[i] + d*lap;
            }
            input = output;
        }
    }
    
    void normals3D(std::vector<CGLA::Vec3f>& normals,
                   std::vector<CGLA::Vec3f>& tangents,
                   std::vector<CGLA::Vec3f>& skeleton)
    {
        for (int i = 0; i < skeleton.size(); i++) {
            Vec3f t;
            if (i == 0) {
                t = skeleton[1]-skeleton[0];
            } else if (i == (skeleton.size() - 1)) {
                t = skeleton[i]-skeleton[i-1];
            } else {
                Vec3f v1 = skeleton[i]-skeleton[i-1];
                Vec3f v2 = skeleton[i+1]-skeleton[i];
                t = 0.5f*(v1+v2);
            }
            tangents.push_back(normalize(t));
        }
        
        Vec3f lastTangent = tangents[0];
        Vec3f lastNorm = orthogonalVectorTo(tangents[0]);
        
        for (int i = 0; i < skeleton.size(); i++) {
            Vec3f tangent = tangents[i];
            Quatf q;
            q.make_rot(lastTangent, tangent);
            Vec3f curNorm = q.apply(lastNorm);
            Vec3f curNormGLK = normalize(curNorm);
            normals.push_back(curNormGLK);
            lastTangent = tangent;
            lastNorm = curNorm;
        }
    }
    
    CGLA::Vec3f orthogonalVectorTo(CGLA::Vec3f vector)
    {
        float x = 1;
        float y = 1;
        float z;
        if (vector[2] == 0) {
            x = 0;
            y = 0;
            z = 1;
        } else {
            z = (vector[0]*x + vector[1]*y) / -vector[2];
        }
        
        Vec3f orthoVector = normalize(Vec3f(x, y, z));
        return orthoVector;
    }

}








































