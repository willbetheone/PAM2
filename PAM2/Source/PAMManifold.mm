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
#include <GLKit/GLKMath.h>
#include <limits.h>
#include "../HMesh/obj_load.h"

namespace PAMMesh
{
    using namespace HMesh;
    using namespace RAEngine;
    using namespace CGLA;
    using namespace std;
    
    int VERTEX_SIZE =  3 * sizeof(float);
    int COLOR_SIZE =  4 * sizeof(unsigned char);
    int INDEX_SIZE  = sizeof(unsigned int);
    
#pragma mark - CONSTRUCTOR/DESTRUCTOR
    PAMManifold::PAMManifold() : HMesh::Manifold() {}

#pragma mark - INHERITED VIRTUAL FUNCTTIONS
    int PAMManifold::loadObjFile(const char *path)
    {
        if (!HMesh::obj_load(path, *this)) {
            RA_LOG_ERROR("Failed to load obj file %s", path);
            return 0;
        }

        return 1;
    }
    
    RABoundingBox PAMManifold::getBoundingBox()
    {
        Vec3d pmin;
        Vec3d pmax;
        HMesh::bbox(*this, pmin, pmax);
        
        RABoundingBox boundingBox;
        boundingBox.minBound = Vec3f(pmin);
        boundingBox.maxBound = Vec3f(pmax);
        
        boundingBox.center =  0.5f * (boundingBox.minBound + boundingBox.maxBound);
        
        Vec3f mid = 0.5f * (boundingBox.maxBound - boundingBox.minBound);
        boundingBox.radius = mid.length();
        boundingBox.width = fabsf(boundingBox.maxBound[0] - boundingBox.minBound[0]);
        boundingBox.height = fabsf(boundingBox.maxBound[1] - boundingBox.minBound[1]);
        boundingBox.depth = fabsf(boundingBox.maxBound[2] - boundingBox.minBound[2]);
        
        return boundingBox;
    }
    
    
#pragma mark - PUBLIC FUNCTIONS
    
    void PAMManifold::setupShaders()
    {
        std::string vShader_Cplus([[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"vsh"].UTF8String);
        std::string fShader_Cplus([[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"fsh"].UTF8String);
        
        drawShaderProgram = new RAES2ShaderProgram();
        drawShaderProgram->loadProgram(vShader_Cplus, fShader_Cplus);
        
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("aPosition");
        attrib[ATTRIB_NORMAL] = drawShaderProgram->getAttributeLocation("aNormal");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("aColor");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
        uniforms[UNIFORM_NORMAL_MATRIX] = drawShaderProgram->getUniformLocation("uNormalMatrix");
    }
   
    void PAMManifold::bufferVertexDataToGPU()
    {
        CGLA::Vec3f* vertexPositions;
        CGLA::Vec3f* vertexNormals;
        CGLA::Vec4uc* vertexColors;
        std::vector<unsigned int>* indicies;
        getVertexData(vertexPositions, vertexNormals, vertexColors, indicies);

        assert(no_vertices() < std::numeric_limits<GLsizei>::max()); //narrowing size_t -> GLsize
        numVerticies = (GLsizei)no_vertices();
        
        positionDataBuffer = new RAES2VertexBuffer(VERTEX_SIZE,
                                                      numVerticies,
                                                      vertexPositions,
                                                      GL_DYNAMIC_DRAW,
                                                      GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(attrib[ATTRIB_POSITION]);
        
        normalDataBuffer = new RAES2VertexBuffer(VERTEX_SIZE,
                                                    numVerticies,
                                                    vertexNormals,
                                                    GL_STATIC_DRAW,
                                                    GL_ARRAY_BUFFER);
        normalDataBuffer->enableAttribute(attrib[ATTRIB_NORMAL]);
        
        colorDataBuffer = new RAES2VertexBuffer(COLOR_SIZE,
                                                   numVerticies,
                                                   vertexColors,
                                                   GL_STATIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        colorDataBuffer->enableAttribute(attrib[ATTRIB_COLOR]);

        assert(indicies->size() < std::numeric_limits<GLsizei>::max()); //narrowing size_t -> GLsize
        numIndicies = (GLsizei)indicies->size();
        
        indexDataBuffer = new RAES2VertexBuffer(INDEX_SIZE,
                                                   numIndicies,
                                                   indicies->data(),
                                                   GL_STATIC_DRAW,
                                                   GL_ELEMENT_ARRAY_BUFFER);
        delete[] vertexPositions;
        delete[] vertexNormals;
        delete[] vertexColors;
        delete indicies;
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
        Vec4uc color(200,200,200,255);

        vertexPositions = new Vec3f[no_vertices()];
        vertexNormals = new Vec3f[no_vertices()];
        vertexColors = new Vec4uc[no_vertices()];
        indicies = new vector<unsigned int>();
        
        int i = 0;
        for (VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid, ++i) {
            assert((*vid).index < no_vertices());
            
            vertexPositions[i] = posf(*vid);
            vertexNormals[i] = HMesh::normalf(*this, *vid);
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
        Mat3x3 normalMat = transpose(getNormalMatrix());
        
        glPushGroupMarkerEXT(0, "Drawing PAM");
        
        glUseProgram(drawShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;
        glUniformMatrix3fv(uniforms[UNIFORM_NORMAL_MATRIX], 1, 0, normalMat.get());
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