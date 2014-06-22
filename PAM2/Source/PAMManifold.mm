//
//  PAMManifold.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "PAMManifold.h"
#include "Vec4uc.h"
#include "RALogManager.h"
#include <OpenGLES/ES2/glext.h>

namespace PAMMesh
{
    using namespace HMesh;
    using namespace RAEngine;
    using namespace CGLA;
    using namespace std;
    
    int VERTEX_SIZE =  3 * sizeof(float);
    int COLOR_SIZE =  4 * sizeof(unsigned char);
    int INDEX_SIZE  = sizeof(unsigned int);
    
    PAMManifold::PAMManifold() : HMesh::Manifold()
    {
    }
    
    void PAMManifold::setupShaders()
    {
        drawShaderProgram = new RAES2ShaderProgram();
        NSString* vShader = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"vsh"];
        NSString* fShader = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"fsh"];
        std::string vShader_Cplus([vShader UTF8String]);
        std::string fShader_Cplus([fShader UTF8String]);
        
        drawShaderProgram->loadProgram(vShader_Cplus, fShader_Cplus);

        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("position");
        attrib[ATTRIB_NORMAL] = drawShaderProgram->getAttributeLocation("normal");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("color");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("modelViewProjectionMatrix");
        uniforms[UNIFORM_NORMAL_MATRIX] = drawShaderProgram->getUniformLocation("normalMatrix");
    }
   
    void PAMManifold::bufferVertexDataToGPU()
    {
        CGLA::Vec3f* vertexPositions;
        CGLA::Vec3f* vertexNormals;
        CGLA::Vec4uc* vertexColors;
        std::vector<unsigned int>* indicies;
        getVertexData(vertexPositions, vertexNormals, vertexColors, indicies);
        numVerticies = no_vertices();
        
        positionDataBuffer = new RAVertexAttribBuffer(VERTEX_SIZE,
                                                      numVerticies,
                                                      vertexPositions,
                                                      GL_DYNAMIC_DRAW,
                                                      GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(ATTRIB_POSITION);
        
        normalDataBuffer = new RAVertexAttribBuffer(VERTEX_SIZE,
                                                    numVerticies,
                                                    vertexNormals,
                                                    GL_STATIC_DRAW,
                                                    GL_ARRAY_BUFFER);
        normalDataBuffer->enableAttribute(ATTRIB_NORMAL);
        
        colorDataBuffer = new RAVertexAttribBuffer(COLOR_SIZE,
                                                   numVerticies,
                                                   vertexColors,
                                                   GL_STATIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        colorDataBuffer->enableAttribute(ATTRIB_COLOR);
        
        numIndicies = indicies->size();
        
        indexDataBuffer = new RAVertexAttribBuffer(INDEX_SIZE,
                                                   numIndicies,
                                                   indicies->data(),
                                                   GL_STATIC_DRAW,
                                                   GL_ELEMENT_ARRAY_BUFFER);
    }
    
    void PAMManifold::normalizeVertexCoordinates()
    {
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
                                    std::vector<unsigned int>*& indicies) const
    {
        Vec4uc color(200,0,0,255);

        vertexPositions = new Vec3f[no_vertices()];
        vertexNormals = new Vec3f[no_vertices()];
        vertexColors = new Vec4uc[no_vertices()];
        indicies = new vector<unsigned int>();
        
        int i = 0;
        for (VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid, ++i) {
            assert((*vid).index < no_vertices());
            
            vertexPositions[i] = posf(*vid);;
            vertexNormals[i] = HMesh::normalf(*this, *vid);
//            RA_LOG_INFO("normal %f,%f,%f", vertexNormals[i][0],vertexNormals[i][1], vertexNormals[i][2]);
            vertexColors[i] = color;
        }
        
        for (FaceIDIterator fid = faces_begin(); fid != faces_end(); ++fid) {
            int vertexNum = 0;
            unsigned int facet[4];
            
            //iterate over every vertex of the face
            for (Walker w = walker(*fid); !w.full_circle(); w = w.circulate_face_ccw()) {
                //add vertex to the data array
                VertexID vID = w.vertex();
                unsigned int index = vID.index;
                
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
    
    void PAMManifold::draw()
    {
        
        Mat4x4 mvpMat = transpose(getModelViewProjectionMatrix());
        Mat3x3 normal = transpose(getNormalMatrix());
        
        glPushGroupMarkerEXT(0, "Drawing PAM");
        
        glUseProgram(drawShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;
        glUniformMatrix3fv(uniforms[UNIFORM_NORMAL_MATRIX], 1, 0, normal.get());
        GL_CHECK_ERROR;
        
        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attrib[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);

        normalDataBuffer->bind();
        normalDataBuffer->prepareToDraw(attrib[ATTRIB_NORMAL], 3, 0, GL_FLOAT, GL_FALSE);
        
        colorDataBuffer->bind();
        colorDataBuffer->prepareToDraw(attrib[ATTRIB_COLOR], 4, 0, GL_UNSIGNED_BYTE, GL_TRUE);
        
        indexDataBuffer->bind();
        indexDataBuffer->drawPreparedArraysIndicies(GL_TRIANGLES, GL_UNSIGNED_INT, numIndicies);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();
    }
}