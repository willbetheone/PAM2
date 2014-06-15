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
    }
    
    RAMesh::~RAMesh()
    {
        delete positionDataBuffer;
        delete normalDataBuffer;
        delete colorDataBuffer;
        delete indexDataBuffer;
    }
    
    Mat4x4 RAMesh::getModelViewProjectionMatrix()
    {
        return projectionMatrix * getModelViewMatrix();
    }
    
    Mat4x4 RAMesh::getModelViewMatrix()
    {
        return viewMatrix * getModelMatrix();
    }
    
    Mat4x4 RAMesh::getModelMatrix()
    {
        return identity_Mat4x4f();
    }
    
    Mat3x3 RAMesh::getNormalMatrix()
    {
        return transpose(invert(get_Mat3x3f(getModelMatrix())));
    }
    
}