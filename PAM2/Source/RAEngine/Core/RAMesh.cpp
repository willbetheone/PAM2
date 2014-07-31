//
//  RAMesh.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAMesh.h"
#include <limits>


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
        
        enabled = false;
        
        numVerticies = 0;
        numIndicies = 0;
        numWireframeIndicies = 0;
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
        
        //Indexed wireframe data
        delete wireframeColorBuffer;
        delete wireframeIndexBuffer;
        
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
    
    void RAMesh::rotate(float radians, Vec3 axis, Vec3 toPivot)
    {
        Mat4x4 toOrigin = translation_Mat4x4f(toPivot);
        Mat4x4 fromOrigin = translation_Mat4x4f(-1 * toPivot);
        Vec3f modelAxis = Vec3f(invert_ortho(getModelViewMatrix()) * Vec4f(axis, 0));
        Mat4x4 rotMat = rotation_Mat4x4f(modelAxis, radians);
        
        rotationMatrix = fromOrigin * rotMat * toOrigin * rotationMatrix;
    }
    
    void RAMesh::rotate(Mat4x4 mat)
    {
        rotationMatrix = mat * rotationMatrix;
    }
    
    void RAMesh::translate(Vec3 translation)
    {
        translationMatrix =  translation_Mat4x4f(translation) * translationMatrix;
    }
    
    void RAMesh::scale(Vec3 scale, Vec3 toPivot)
    {
//        Mat4x4 toOrigin = translation_Mat4x4f(toPivot);
//        Mat4x4 fromOrigin = translation_Mat4x4f(-1 * toPivot);
        Mat4x4 scaleMat = scaling_Mat4x4f(scale);
        
//        scaleMatrix = fromOrigin * scaleMat * toOrigin * scaleMatrix;
        scaleMatrix = scaleMat;
    }
    
    int RAMesh::loadObjFile(const char* path)
    {
        return 0;
    }
    
    bool RAMesh::testBoundingBoxIntersection (const Vec3& origin, const Vec3& direction,
                                              float tmin, float tmax) const
    {
        Bounds bounds = getBoundingBox();
        float mRadius = bounds.radius;
        Vec3 mCenter = bounds.center;
        
        if (bounds.radius == 0.0f)
        {
            // The bound is invalid and cannot be intersected.
            return false;
        }
        
        Vec3 diff;
        float a0, a1, discr;
        
        if (tmin == -FLT_MAX)
        {
            assert(tmax == FLT_MAX); // tmax must be infinity for a line
            
            // Test for sphere-line intersection.
            diff = origin - mCenter;
            a0 = dot(diff,diff) - mRadius*mRadius;
            a1 = dot(direction,diff);
            discr = a1*a1 - a0;
            return discr >= 0.0f;
        }
        
        if (tmax == FLT_MAX)
        {
            assert(tmin == 0.0f); //tmin must be zero for a ray
            
            // Test for sphere-ray intersection.
            Vec3 diff = origin - mCenter;
            a0 = dot(diff,diff) - mRadius*mRadius;
            if (a0 <= 0.0f)
            {
                // The ray origin is inside the sphere.
                return true;
            }
            // else: The ray origin is outside the sphere.
            
            a1 = dot(direction,diff);
            if (a1 >= 0.0f)
            {
                // The ray forms an acute angle with diff, and so the ray is
                // directed from the sphere.  Thus, the ray origin is outside
                // the sphere, and points P+t*D for t >= 0 are even farther
                // away from the sphere.
                return false;
            }
            
            discr = a1*a1 - a0;
            return discr >= 0.0f;
        }
        
        assert(tmax > tmin); //tmin < tmax is required for a segment
        
        // Test for sphere-segment intersection.
        float segExtent = 0.5f*(tmin + tmax);
        Vec3 segOrigin = origin + segExtent*direction;
        
        diff = segOrigin - mCenter;
        a0 = dot(diff,diff) - mRadius*mRadius;
        a1 = dot(direction,diff);
        discr = a1*a1 - a0;
        if (discr < 0.0f)
        {
            return false;
        }
        
        float tmp0 = segExtent*segExtent + a0;
        float tmp1 = 2.0f*a1*segExtent;
        float qm = tmp0 - tmp1;
        float qp = tmp0 + tmp1;
        if (qm*qp <= 0.0f)
        {
            return true;
        }
        
        return qm > 0.0f && fabs(a1) < segExtent;
    }

}