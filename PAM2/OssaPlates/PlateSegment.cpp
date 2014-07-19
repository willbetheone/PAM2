//
//  RAPlateSegment.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-14.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "PlateSegment.h"
#include "Quatf.h"
#include "../HMesh/obj_load.h"

namespace Ossa
{
    using namespace CGLA;
    using namespace std;
    using namespace RAEngine;
    
    PlateSegment::PlateSegment(const Vec3&  center, const Vec3& norm)
    {
        Quatf quat;
        quat.make_rot(Vec3f(0,1,0), norm);
        quat.get_Mat4x4f();
        rotate(quat.get_Mat4x4f());
        translate(center);
    }
    
    void PlateSegment::setCenterAndNormal(const CGLA::Vec3f& center, const CGLA::Vec3f& norm)
    {
        translationMatrix = identity_Mat4x4f();
        rotationMatrix = identity_Mat4x4f();
        Quatf quat;
        quat.make_rot(Vec3f(0,1,0), norm);
        quat.get_Mat4x4f();
        rotate(quat.get_Mat4x4f());
        translate(center);
    }
    
    void PlateSegment::setupShaders(const std::string vertexShader, const std::string fragmentShader)
    {
        
        drawShaderProgram = new RAES2ShaderProgram();
        drawShaderProgram->loadProgram(vertexShader, fragmentShader);
        
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("aPosition");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("aColor");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
    }
    
    int PlateSegment::loadObjFile(const char* path)
    {
        vector<Vec3f> vertices{};
        vector<Vec3f> normals{};
        vector<int> faces{};
        vector<int> indices{};
        
        HMesh::obj_load(path, vertices, normals, faces, indices);
        Vec4uc* colors = new Vec4uc[vertices.size()];
        std::fill_n(colors, vertices.size(), Vec4uc(0,0,255,255));
        
        numVerticies = vertices.size();
        positionDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                   numVerticies,
                                                   vertices.data(),
                                                   GL_STATIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(attrib[ATTRIB_POSITION]);
        
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
    
    void PlateSegment::draw() const
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
    
    Bounds PlateSegment::getBoundingBox() const
    {
        Bounds bounds{};
        return bounds;
    }

    void PlateSegment::resetTranslation()
    {
        translationMatrix = identity_Mat4x4f();
    }
}