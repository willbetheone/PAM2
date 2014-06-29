//
//  RAUnitSphere.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-28.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAUnitSphere.h"
#include "../HMesh/obj_load.h"
#include "Vec4uc.h"
#include <OpenGLES/ES2/glext.h>
#include <OpenGLES/ES2/gl.h>
#include "RALogManager.h"

namespace RAEngine
{
    using namespace CGLA;
    using namespace std;
    
    RAUnitSphere::RAUnitSphere() : RAUnitSphere(Vec3f(0,0,0))
    {
    }
    
    
    RAUnitSphere::RAUnitSphere(const CGLA::Vec3f& point)
    {
        translate(point);
    }
    
    void RAUnitSphere::setupShaders(const std::string vertexShader, const std::string fragmentShader)
    {
        drawShaderProgram = new RAES2ShaderProgram();
        drawShaderProgram->loadProgram(vertexShader, fragmentShader);
        
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("aPosition");
//        attrib[ATTRIB_NORMAL] = drawShaderProgram->getAttributeLocation("aNormal");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("aColor");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
//        uniforms[UNIFORM_NORMAL_MATRIX] = drawShaderProgram->getUniformLocation("uNormalMatrix");
    }

    int RAUnitSphere::loadObjFile(const char* path)
    {
        vector<Vec3f> vertices{};
        vector<Vec3f> normals{};
        vector<int> faces{};
        vector<int> indices{};
        
        HMesh::obj_load(path, vertices, normals, faces, indices);
        Vec4uc* colors = new Vec4uc[vertices.size()];
        std::fill_n(colors, vertices.size(), Vec4uc(255,0,0,255));
        
        numVerticies = vertices.size();
        positionDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                   numVerticies,
                                                   vertices.data(),
                                                   GL_STATIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(attrib[ATTRIB_POSITION]);
        
//        normalDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
//                                                 numVerticies,
//                                                 normals.data(),
//                                                 GL_STATIC_DRAW,
//                                                 GL_ARRAY_BUFFER);
//        normalDataBuffer->enableAttribute(attrib[ATTRIB_NORMAL]);
        
        colorDataBuffer = new RAES2VertexBuffer(sizeof(Vec4uc),
                                                numVerticies,
                                                colors,
                                                GL_STATIC_DRAW,
                                                GL_ARRAY_BUFFER);
        colorDataBuffer->enableAttribute(attrib[ATTRIB_COLOR]);
        
        numIndicies = indices.size();
        indexDataBuffer = new RAES2VertexBuffer(sizeof(unsigned int),
                                                numIndicies,
                                                indices.data(),
                                                GL_STATIC_DRAW,
                                                GL_ELEMENT_ARRAY_BUFFER);
        
        delete[] colors;
        return 1;
    }
    
    
    void RAUnitSphere::draw() const
    {
        Mat4x4 mvpMat = transpose(getModelViewProjectionMatrix());
//        Mat3x3 normal = transpose(getNormalMatrix());
        
        glPushGroupMarkerEXT(0, "Drawing Unit Sphere");
        
        glUseProgram(drawShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;
//        glUniformMatrix3fv(uniforms[UNIFORM_NORMAL_MATRIX], 1, 0, normal.get());
//        GL_CHECK_ERROR;
        
        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attrib[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        
//        normalDataBuffer->bind();
//        normalDataBuffer->prepareToDraw(attrib[ATTRIB_NORMAL], 3, 0, GL_FLOAT, GL_FALSE);

        colorDataBuffer->bind();
        colorDataBuffer->prepareToDraw(attrib[ATTRIB_COLOR], 4, 0, GL_UNSIGNED_BYTE, GL_TRUE);
        
        indexDataBuffer->bind();
        indexDataBuffer->drawPreparedArraysIndicies(GL_TRIANGLES, GL_UNSIGNED_INT, numIndicies);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();
    }
    
    Bounds RAUnitSphere::getBoundingBox() const
    {
        return {};
    }
}