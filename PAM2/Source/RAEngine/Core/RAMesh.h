//
//  RAMesh.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAMesh__
#define __PAM2__RAMesh__

#include <iostream>
#import <OpenGLES/ES2/gl.h>
#include "Mat4x4f.h"
#include "Mat3x3f.h"
#include "RAES2VertexBuffer.h"
#include "RAES2ShaderProgram.h"
#include "RAES2VertexArray.h"
#include "RABoundingBox.h"

namespace RAEngine
{
    typedef CGLA::Mat4x4f Mat4x4;
    typedef CGLA::Mat3x3f Mat3x3;
    
    class RAMesh
    {
    public:
        RAMesh();
        ~RAMesh();

        Mat4x4 viewMatrix;
        Mat4x4 projectionMatrix;

        GLsizei numVerticies;
        GLsizei numIndicies;
        
        Mat4x4 getModelViewProjectionMatrix();
        Mat4x4 getModelViewMatrix();
        Mat4x4 getModelMatrix();
        Mat3x3 getNormalMatrix();
        
        ///override in the child
        virtual RABoundingBox getBoundingBox() = 0;
        
        ///override in the child
        virtual int loadObjFile(const char* path) = 0;
        
    private:
    
    protected:
        
        enum
        {
            UNIFORM_MODELVIEWPROJECTION_MATRIX,
            UNIFORM_MODELVIEW_MATRIX,
            UNIFORM_MODEL_MATRIX,
            UNIFORM_NORMAL_MATRIX,
            UNIFORM_TEXTURE,
            UNIFORM_LIGHT_POSITION,
            UNIFORM_LIGHT_DIRECTION,
            UNIFORM_LIGHT_COLOR,
            UNIFORM_POINT_SIZE,
            NUM_UNIFORMS
        };
        
        enum
        {
            ATTRIB_COLOR,
            ATTRIB_POSITION,
            ATTRIB_TEXT,
            ATTRIB_NORMAL,
            NUM_ATTRIB
        };
      
        //Display Shader Variables
        GLint attrib[NUM_ATTRIB];
        GLint uniforms[NUM_UNIFORMS];
        
        //Off Screen Depth Shader Variables
        GLint attribDepth[NUM_ATTRIB];
        GLint uniformsDepth[NUM_UNIFORMS];
        
        //Indexed vertex data
        RAES2VertexBuffer* positionDataBuffer;
        RAES2VertexBuffer* normalDataBuffer;
        RAES2VertexBuffer* colorDataBuffer;
        RAES2VertexBuffer* indexDataBuffer;
        
        //Interlieved vertex data
        RAES2VertexBuffer* vertexDataBuffer;
        
        //Vertex array
        RAES2VertexArray* vertexArray;
        
        //Shaders
        RAES2ShaderProgram* drawShaderProgram;
        RAES2ShaderProgram* depthShaderProgram;
        
    };
}

#endif /* defined(__PAM2__RAMesh__) */
