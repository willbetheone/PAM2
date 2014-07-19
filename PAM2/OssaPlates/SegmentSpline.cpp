//
//  PlateSpline.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-14.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "SegmentSpline.h"
#include "RAPolylineUtilities.h"
#include "RALogManager.h"

namespace Ossa {
    
    using namespace std;
    using namespace CGLA;
    using namespace RAEngine;
    
    SegmentSpline::SegmentSpline(std::vector<CGLA::Vec3f> sampleData, float segmentLength)
    {
        controlPoints =  new vector<Vec3f>();
        reduceLineToEqualSegments(*controlPoints, sampleData, segmentLength);
        this->segmentLength = segmentLength;
    }

    SegmentSpline::~SegmentSpline()
    {
        delete controlPoints;
    }
    
    int SegmentSpline::getControlQuantity() const
    {
        return controlPoints->size();
    }
    
    void SegmentSpline::setControlPoint(int i, const CGLA::Vec3f& ctrl)
    {
        if (i > controlPoints->size()) {
            RA_LOG_WARN("control point index %i is out of bounds", i);
            return;
        }
        
        (*controlPoints)[i] = ctrl;
    }
    
    CGLA::Vec3f SegmentSpline::getControlPoint(int i) const
    {
        if (i > controlPoints->size()) {
            RA_LOG_WARN("control point index %i is out of bounds", i);
            return Vec3f(0,0,0);
        }
        
        return (*controlPoints)[i];
    }
    
    void SegmentSpline::getControlPoints(std::vector<CGLA::Vec3f>& points)
    {
        for (Vec3f p : *controlPoints) {
            points.push_back(p);
        }
    }
    
    void SegmentSpline::getSpline(std::vector<CGLA::Vec3f>& splineData) const
    {
        reduceLineToEqualSegments(splineData, *controlPoints, segmentLength);
    }
    
    void SegmentSpline::setSampleData(std::vector<CGLA::Vec3f> sampleData, int controlPoint)
    {
        vector<Vec3f> newControlPoints;
        reduceLineToEqualSegments(newControlPoints, sampleData, segmentLength);
        controlPoints->erase(controlPoints->begin() + controlPoint + 1, controlPoints->end());
        controlPoints->insert(controlPoints->end(), newControlPoints.begin(), newControlPoints.end());
    }
}