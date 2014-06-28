//
//  RAMesh.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAMesh.h"

namespace RAEngine {

    using namespace CGLA;
    
#pragma mark - Public methods
    RAMesh::RAMesh()
    {
        viewMatrix = identity_Mat4x4f();
        projectionMatrix = identity_Mat4x4f();
        
        translationMatrix = identity_Mat4x4f();
        rotationMatrix = identity_Mat4x4f();
        scaleMatrix = identity_Mat4x4f();
    }
    
    RAMesh::~RAMesh()
    {
        //Indexed vertex data
        delete positionDataBuffer;
        delete normalDataBuffer;
        delete colorDataBuffer;
        delete indexDataBuffer;

        //Interlieved vertex data
        delete vertexDataBuffer;
        
        //Vertex array
        delete vertexArray;

        //Shaders
        delete drawShaderProgram;
        delete depthShaderProgram;
    }
    
    Mat4x4 RAMesh::getModelViewProjectionMatrix() const
    {
        return projectionMatrix * getModelViewMatrix();
    }
    
    Mat4x4 RAMesh::getModelViewMatrix() const
    {
        return viewMatrix * getModelMatrix();
    }
    
    Mat4x4 RAMesh::getModelMatrix() const
    {
        Mat4x4 modelMatrix = identity_Mat4x4f();
        modelMatrix = translationMatrix * modelMatrix ;
        modelMatrix = rotationMatrix * modelMatrix  ;
        //        modelMatrix = translationManager->getTranslationMatrix() * modelMatrix;
        return modelMatrix;
    }
    
    Mat3x3 RAMesh::getNormalMatrix() const
    {
        return transpose(invert(get_Mat3x3f(getModelViewMatrix())));
    }
    
    void RAMesh::rotate(float radians, Vec3 axis)
    {
        Vec3f modelAxis = Vec3f(invert_ortho(getModelViewMatrix()) * Vec4f(axis, 0));
        Mat4x4f rm = rotation_Mat4x4f(modelAxis, radians);
        rotationMatrix = rotationMatrix * rm;
    }
    
    void RAMesh::rotate(float radians, Vec3 axis, Vec3 toOriginVec)
    {
        
        Mat4x4 toOrigin = translation_Mat4x4f(toOriginVec);
        Mat4x4 fromOrigin = translation_Mat4x4f(-1 * toOriginVec);
        Mat4x4 rotMat = rotation_Mat4x4f(axis, radians);
        
        rotationMatrix = rotationMatrix*fromOrigin * rotMat * toOrigin;
    }
    
    void RAMesh::translate(Vec3 translation)
    {
        translationMatrix =  translationMatrix * translation_Mat4x4f(translation);
    }
    
    int RAMesh::loadObjFile(const char* path)
    {
        return 0;
    }

}