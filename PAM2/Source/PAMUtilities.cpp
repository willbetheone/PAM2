//
//  PAMUtilities.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-23.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "PAMUtilities.h"
#include "RAPolylineUtilities.h"
#include "RALogManager.h"

using namespace std;
using namespace CGLA;
using namespace RAEngine;

#define SHOULD LOG 0

namespace PAMMesh
{
    bool getRibWidths(const std::vector<CGLA::Vec2f>& line1,
                      const std::vector<CGLA::Vec2f>& line2,
                      const std::vector<CGLA::Vec2f>& center,
                      const std::vector<CGLA::Vec2f>& norms,
                      std::vector<float>& ribWidths)
    {
        assert(center.size() > 3);
        assert(center.size() == norms.size());

        for (int i = 0; i < center.size(); ++i)
        {
            Vec2f q = center[i];
            Vec2f s = norms[i];

            vector<float> minrULine1;
            for (int j = 0; j < line1.size() - 1; ++j)
            {
                Vec2f p1 = line1[j];
                Vec2f p2 = line1[j+1];
                
                float rU;
                if (lineSegmentRayIntersection(p1, p2, q, s, rU)) {
                    minrULine1.push_back(rU);
                }
                
                float rU2;
                if (lineSegmentRayIntersection(p1, p2, q, -s, rU2)) {
                    minrULine1.push_back(rU2);
                }
            }
            
            vector<float> minrULine2;
            for (int j = 0; j < line2.size() - 1; ++j)
            {
                Vec2f p1 = line2[j];
                Vec2f p2 = line2[j+1];
                
                float rU;
                if (lineSegmentRayIntersection(p1, p2, q, s, rU)) {
                    minrULine2.push_back(rU);
                }
                
                float rU2;
                if (lineSegmentRayIntersection(p1, p2, q, -s, rU2)) {
                    minrULine2.push_back(rU2);
                }
            }
            
            if (i == center.size() - 1) {
                RA_LOG_INFO("STOP HERe");
            }
            
            if (minrULine1.size() > 0 || minrULine2.size() > 0) {
            
                sort(minrULine1.begin(), minrULine1.end());
                sort(minrULine2.begin(), minrULine2.end());
                
                float rU;
                if (minrULine1.size() > 0 && minrULine2.size() > 0) {
                    rU = min(minrULine1[0], minrULine2[0]);
                } else if (minrULine1.size() > 0) {
                    rU = minrULine1[0];
                } else if (minrULine2.size() > 0) {
                    rU = minrULine2[0];
                }
                
                RA_LOG_INFO("Width for skeleton index: %f %i", rU, i);
                ribWidths.push_back(rU);
            } else{
                if (ribWidths.size() > 0) {
                    ribWidths.push_back(ribWidths[ribWidths.size() -1]); //append last one
                    RA_LOG_INFO("Used previous width for %i", i);
                } else {
                    ribWidths.push_back(0);
                    RA_LOG_INFO("Used ZERO width for %i", i);
                }
            }

        }
        
        return true;
    }
    


}
