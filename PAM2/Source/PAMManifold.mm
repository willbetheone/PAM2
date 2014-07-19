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
#include <map>


namespace PAMMesh
{
    using namespace HMesh;
    using namespace RAEngine;
    using namespace CGLA;
    using namespace std;
    using namespace Geometry;
    
#pragma mark - CONSTRUCTOR/DESTRUCTOR
    PAMManifold::PAMManifold() : HMesh::Manifold() {}
    PAMManifold::~PAMManifold()
    {
        delete kdTree;
    }

#pragma mark - INHERITED VIRTUAL FUNCTTIONS
    int PAMManifold::loadObjFile(const char *path)
    {
        if (!HMesh::obj_load(path, *this)) {
            RA_LOG_ERROR("Failed to load obj file %s", path);
            return 0;
        }
        return 1;
    }
    
    RAEngine::Bounds PAMManifold::getBoundingBox() const
    {
        Vec3d pmin;
        Vec3d pmax;
        HMesh::bbox(*this, pmin, pmax);
        
        Vec3d mid = pmax - pmin;
        float radius = 0.5*mid.length();
        Vec3d center =  pmin + radius*normalize(mid);
        
        Bounds bnds = {Vec3(pmin), Vec3(pmax), Vec3(center), radius};
        return bnds;
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
        Vec3f* vertexPositions;
        Vec3f* vertexNormals;
        Vec4uc* vertexColors;
        std::vector<unsigned int>* indicies;
        getVertexData(vertexPositions, vertexNormals, vertexColors, indicies);

        assert(no_vertices() < std::numeric_limits<GLsizei>::max()); //narrowing size_t -> GLsize
        numVerticies = (GLsizei)no_vertices();
        
        positionDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                   numVerticies,
                                                   vertexPositions,
                                                   GL_DYNAMIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(attrib[ATTRIB_POSITION]);
        
        normalDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                 numVerticies,
                                                 vertexNormals,
                                                 GL_STATIC_DRAW,
                                                 GL_ARRAY_BUFFER);
        normalDataBuffer->enableAttribute(attrib[ATTRIB_NORMAL]);
        
        colorDataBuffer = new RAES2VertexBuffer(sizeof(Vec4uc),
                                                numVerticies,
                                                vertexColors,
                                                GL_STATIC_DRAW,
                                                GL_ARRAY_BUFFER);
        colorDataBuffer->enableAttribute(attrib[ATTRIB_COLOR]);

        assert(indicies->size() < std::numeric_limits<GLsizei>::max()); //narrowing size_t -> GLsize
        numIndicies = (GLsizei)indicies->size();
        
        indexDataBuffer = new RAES2VertexBuffer(sizeof(unsigned int),
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
        
        // when interating through faces we need to map VertexID to index
        std::map<VertexID, int>* vertexIDtoIndex = new std::map<VertexID, int>();
        
        int i = 0;
        for (VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid, ++i) {
//            assert((*vid).index < no_vertices());
            
            vertexPositions[i] = posf(*vid);
            vertexNormals[i] = HMesh::normalf(*this, *vid);
            vertexColors[i] = color;
            (*vertexIDtoIndex)[*vid] = i;
        }
        
        for (FaceIDIterator fid = faces_begin(); fid != faces_end(); ++fid) {
            int vertexNum = 0;
            unsigned int facet[4];
            
            //iterate over every vertex of the face
            for (Walker w = walker(*fid); !w.full_circle(); w = w.circulate_face_ccw()) {
                //add vertex to the data array
                VertexID vID = w.vertex();
//                unsigned int index = vID.index;
                unsigned int index = (*vertexIDtoIndex)[vID];
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
        delete vertexIDtoIndex;

    }
    
    bool PAMManifold::normal(const CGLA::Vec3f& point, CGLA::Vec3f& norm)
    {
        VertexID vid;
        if (closestVertexID_3D(point, vid)){
            norm = normalf(*this, vid);
            return true;
        }
        return false;
    }
    
    bool PAMManifold::closestVertexID_3D(const CGLA::Vec3f& point, HMesh::VertexID& vid)
    {
        if (kdTree != nullptr) {
            Vec3f coord;
            Vec3f pointModel = invert_affine(getModelMatrix()).mul_3D_point(point);
            float max = 0.1;
            if (kdTree->closest_point(pointModel, max, coord, vid)){
                return true;
            }
            return false;
        } else {
            //iterate over every face
            float distance = FLT_MAX;
            HMesh::VertexID closestVertex;
            Vec3f pointModel = Vec3f(invert_affine(getModelMatrix()) * Vec4f(point));
            
            if (no_vertices()==0) {
                return false;
            }

            for (VertexIDIterator vID = vertices_begin(); vID != vertices_end(); vID++)
            {
                Vec3f vertexPos = posf(*vID);
                float cur_distance = (pointModel - vertexPos).length();
                if (cur_distance < distance) {
                    distance = cur_distance;
                    vid = *vID;
                }
            }
            return true;
        }
    }
    
    void PAMManifold::buildKDTree()
    {
        kdTree = new KDTree<Vec3f, VertexID>();
        for(VertexIDIterator vid = vertices_begin(); vid != vertices_end(); ++vid) {
            kdTree->insert(posf(*vid), *vid);
        }
        kdTree->build();
    }
    
    void PAMManifold::draw() const
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
    
    void PAMManifold::drawToDepthBuffer() 
    {
        if (depthShaderProgram == nullptr) {
            std::string vShader_Cplus([[NSBundle mainBundle] pathForResource:@"DepthShader" ofType:@"vsh"].UTF8String);
            std::string fShader_Cplus([[NSBundle mainBundle] pathForResource:@"DepthShader" ofType:@"fsh"].UTF8String);
            
            depthShaderProgram = new RAES2ShaderProgram();
            depthShaderProgram->loadProgram(vShader_Cplus, fShader_Cplus);
            
            attribDepth[ATTRIB_POSITION] = depthShaderProgram->getAttributeLocation("aPosition");
            uniformsDepth[UNIFORM_MODELVIEWPROJECTION_MATRIX] = depthShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
        }
        
        Mat4x4 mvpMat = transpose(getModelViewProjectionMatrix());
        glPushGroupMarkerEXT(0, "Drawing PAM to Depth Buffer");
        
        glUseProgram(depthShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniformsDepth[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;
        
        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attribDepth[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        
        indexDataBuffer->bind();
        indexDataBuffer->drawPreparedArraysIndicies(GL_TRIANGLES, GL_UNSIGNED_INT, numIndicies);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();
    }

}