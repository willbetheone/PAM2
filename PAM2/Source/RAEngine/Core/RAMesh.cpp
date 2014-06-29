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
        //TODO check the order!
        Mat4x4 modelMatrix = translationMatrix*rotationMatrix*scaleMatrix;
        return modelMatrix;
    }
    
    Mat3x3 RAMesh::getNormalMatrix() const
    {
        return transpose(invert(get_Mat3x3f(getModelViewMatrix())));
    }
    
//    void RAMesh::rotate(float radians, Vec3 axis)
//    {
//        Vec3f modelAxis = Vec3f(invert_ortho(getModelViewMatrix()) * Vec4f(axis, 0));
//        Mat4x4f rm = rotation_Mat4x4f(modelAxis, radians);
//        rotationMatrix = rm * rotationMatrix;
//    }
    
    void RAMesh::rotate(float radians, Vec3 axis, Vec3 toPivot)
    {
        Mat4x4 toOrigin = translation_Mat4x4f(toPivot);
        Mat4x4 fromOrigin = translation_Mat4x4f(-1 * toPivot);
        Vec3f modelAxis = Vec3f(invert_ortho(getModelViewMatrix()) * Vec4f(axis, 0));
        Mat4x4 rotMat = rotation_Mat4x4f(modelAxis, radians);
        
        rotationMatrix = fromOrigin * rotMat * toOrigin * rotationMatrix;
    }
    
    void RAMesh::translate(Vec3 translation)
    {
        translationMatrix =  translation_Mat4x4f(translation) * translationMatrix;
    }
    
    void RAMesh::scale(Vec3 scale, Vec3 toPivot)
    {
        Mat4x4 toOrigin = translation_Mat4x4f(toPivot);
        Mat4x4 fromOrigin = translation_Mat4x4f(-1 * toPivot);
        Mat4x4 scaleMat = scaling_Mat4x4f(scale);
        
        scaleMatrix = fromOrigin * scaleMat * toOrigin * scaleMatrix;
    }
    
    int RAMesh::loadObjFile(const char* path)
    {
        return 0;
    }

}