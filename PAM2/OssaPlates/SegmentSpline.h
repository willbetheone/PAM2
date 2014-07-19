//
//  PlateSpline.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-14.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__SegmentSpline__
#define __PAM2__SegmentSpline__

#include <iostream>
#include <vector>
#include "Vec3f.h"

namespace Ossa {
    class SegmentSpline
    {
    public:
        SegmentSpline(std::vector<CGLA::Vec3f> sampleData, float segmentLength);
        ~SegmentSpline();
        
        int getControlQuantity() const;
        void setControlPoint(int i, const CGLA::Vec3f& ctrl);
        CGLA::Vec3f getControlPoint(int i) const;
        void getControlPoints(std::vector<CGLA::Vec3f>& points);
        
        void getSpline(std::vector<CGLA::Vec3f>& splineData) const;

        ///change sample data starting from control point give
        void setSampleData(std::vector<CGLA::Vec3f> sampleData, int controlPoint);
        

    private:
        std::vector<CGLA::Vec3f>* controlPoints;
        float segmentLength;
    };
}

#endif /* defined(__PAM2__PlateSpline__) */
