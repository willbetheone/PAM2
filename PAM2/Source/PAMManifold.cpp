//
//  PAMManifold.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "PAMManifold.h"
#include "Vec4uc.h"

namespace PAMMesh {
    using namespace HMesh;
    using namespace CGLA;
    using namespace std;
    
    void PAMManifold::normalizeVertexCoordinates() noexcept {
        //Calculate Bounding Box
        Vec3d pmin;
        Vec3d pmax;
        HMesh::bbox(*this, pmin, pmax);
        
        Vec3d midV = 0.5 * (pmax - pmin);
        float rad = midV.length();
        
        for (VertexID vID: this->vertices()) {
            Vec3d pos = this->pos(vID);
            Vec3d newPos = (pos - pmin - midV) / rad;
            this->pos(vID) = newPos;
        }
    }
    
    void PAMManifold::getVertexData(CGLA::Vec3f*& vertexPositions,
                                    CGLA::Vec3f*& vertexNormals,
                                    CGLA::Vec4uc*& vertexColors,
                                    std::vector<size_t>*& indicies) const noexcept
    {
        Vec4uc color(200,200,200,255);

        vertexPositions = new Vec3f[no_vertices()];
        vertexNormals = new Vec3f[no_vertices()];
        vertexColors = new Vec4uc[no_vertices()];
        indicies = new vector<size_t>();
        
        int i = 0;
        for (VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid, ++i) {
            assert((*vid).index < no_vertices());
            
            vertexPositions[i] = posf(*vid);;
            vertexNormals[i] = HMesh::normalf(*this, *vid);
            vertexColors[i] = *color.get();
        }
        
        for (FaceIDIterator fid = faces_begin(); fid != faces_end(); ++fid) {
            int vertexNum = 0;
            size_t facet[4];
            
            //iterate over every vertex of the face
            for (Walker w = walker(*fid); !w.full_circle(); w = w.circulate_face_ccw()) {
                //add vertex to the data array
                VertexID vID = w.vertex();
                size_t index = vID.index;
                
                assert(index < no_vertices());
                
                facet[vertexNum] = index;
                vertexNum++;
                
                if (vertexNum == 4) {
                    //Create a second triangle
                    indicies->push_back(facet[0]);
                    indicies->push_back(facet[2]);
                }
                indicies->push_back(index);
            }
        }
    }
    
}