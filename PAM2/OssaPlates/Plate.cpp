//
//  Plate.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-17.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "Plate.h"
#include "PlateSegment.h"
#include "Quatf.h"
#include "../HMesh/obj_load.h"

namespace Ossa
{
    using namespace CGLA;
    using namespace RAEngine;
    using namespace std;
    
    void Plate::setSpline(const std::vector<CGLA::Vec3f>& points,
                          const std::vector<CGLA::Vec3f>& normals,
                          const std::vector<CGLA::Vec3f>& tangent)
    {
        assert(points.size() == normals.size());
        assert(points.size() == tangent.size());
        rotMatricies.clear();
        translationMatricies.clear();

        for (int i = 0; i < points.size(); i++)
        {
            //translation
            Mat4x4f fromOrigin = translation_Mat4x4f(points[i]);
            
            //rotation to norm
            Quatf quat;
            quat.make_rot(Vec3f(0,1,0), normals[i]);
            Mat4x4f rotNormMat = quat.get_Mat4x4f();
            
            //rotation to tangent
            //            Vec3f xAxis =  rotNormMat.mul_3D_vector(Vec3f(1,0,0));
            //            Quatf quat2;
            //            quat2.make_rot(xAxis, tangent[i]);
            //            Mat4x4f rotTanMat = quat2.get_Mat4x4f();
            Mat4x4f rotTanMat = identity_Mat4x4f();
            
            rotMatricies.push_back(rotTanMat*rotNormMat);
            translationMatricies.push_back(fromOrigin);
        }

    }
    
    void Plate::setupShaders(const std::string& vertexShader,
                             const std::string& fragmentShader)
    {
        
        drawShaderProgram = new RAES2ShaderProgram();
        drawShaderProgram->loadProgram(vertexShader, fragmentShader);
        
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("aPosition");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("aColor");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
    }
    
    int Plate::loadObjFile(const char* path)
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

    RAEngine::Bounds Plate::getBoundingBox() const
    {
        Bounds b{};
        return b;
        
    }
    void Plate::draw() const
    {
        glPushGroupMarkerEXT(0, "Drawing Plates");
        glUseProgram(drawShaderProgram->getProgram());
        GL_CHECK_ERROR;
        
        Mat4x4 vpMat = projectionMatrix * viewMatrix;
        
        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attrib[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        
        colorDataBuffer->bind();
        colorDataBuffer->prepareToDraw(attrib[ATTRIB_COLOR], 4, 0, GL_UNSIGNED_BYTE, GL_TRUE);
        
        indexDataBuffer->bind();
        
        for (int i = 0; i < rotMatricies.size(); i++)
        {
            Mat4x4 scaleMat = scaling_Mat4x4f(Vec3f(scale, scale, scale));
            Mat4x4 mMat = translationMatricies[i] * rotMatricies[i] * scaleMat;
            Mat4x4 mvpMat = transpose(vpMat*mMat);
            
            glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
            GL_CHECK_ERROR;
            
            indexDataBuffer->drawPreparedArraysIndicies(GL_TRIANGLES, GL_UNSIGNED_INT, numIndicies);            
        }
        
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();
    }
}