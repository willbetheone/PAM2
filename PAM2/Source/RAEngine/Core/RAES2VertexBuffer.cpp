//
//  RAES2VertexBuffer.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAES2VertexBuffer.h"
#include "RALogManager.h"
#include <OpenGLES/ES2/glext.h>
#include <assert.h> 

namespace RAEngine {
    
    RAES2VertexBuffer::RAES2VertexBuffer(GLsizei aStride,
                                         GLsizei count,
                                         const GLvoid* dataPtr,
                                         GLenum usage,
                                         GLenum aTarget)
    {
        assert(0 < aStride);
        assert((NULL != dataPtr) || (0 == count && NULL == dataPtr));
        
        stride = aStride;
        bufferSizeBytes = stride * count;
        target = aTarget;
        
        glGenBuffers(1, &name);
        GL_CHECK_ERROR;
        glBindBuffer(target, name);
        GL_CHECK_ERROR;
        
        glBufferData(target,  // Initialize buffer contents
                     bufferSizeBytes,  // Number of bytes to copy
                     dataPtr,          // Address of bytes to copy
                     usage);           // Hint: cache in GPU memory
        GL_CHECK_ERROR;
        
        assert(0 != name);
    }
    
    GLuint RAES2VertexBuffer::getName() const
    {
        return name;
    }
    
    void RAES2VertexBuffer::bufferSubData(GLintptr offset,
                                          GLsizeiptr size,
                                          const GLvoid* dataPtr)
    {
        assert(offset + size <= bufferSizeBytes);
        glBufferSubData(target, 0, bufferSizeBytes, dataPtr);
        GL_CHECK_ERROR;
    }

    void RAES2VertexBuffer::enableAttribute(GLuint index) const
    {
        glEnableVertexAttribArray(index);
        GL_CHECK_ERROR;
    }
    
    void RAES2VertexBuffer::bind() const
    {
        glBindBuffer(target, name);
        GL_CHECK_ERROR;
    }
    
    void RAES2VertexBuffer::prepareToDraw(GLuint index,
                                          GLint count,
                                          GLsizeiptr offset,
                                          GLenum type,
                                          GLboolean normalized) const
    {
        assert((0 < count) && (count <= 4));
        assert(offset < stride);
        assert(0 != name);
        
        glVertexAttribPointer(index,               // Identifies the attribute to use
                              count,               // number of coordinates for attribute
                              type,                // data is floating point
                              normalized,            // no fixed point scaling
                              stride,         // total num bytes stored per vertex
                              (char*)NULL + offset);      // offset from start of each vertex to
        
        GL_CHECK_ERROR;
    }
    
    void RAES2VertexBuffer::drawPreparedArrays(GLenum mode,
                                               GLint first,
                                               GLsizei count)
    {
        glDrawArrays(mode, first, count);
        GL_CHECK_ERROR;
    }
    
    void RAES2VertexBuffer::drawPreparedArraysIndicies(GLenum mode,
                                                       GLenum dataType,
                                                       GLsizei numIndcies)
    {
        glDrawElements(mode, numIndcies, dataType, 0);
        GL_CHECK_ERROR;
    }

    RAES2VertexBuffer::~RAES2VertexBuffer()
    {
        // Delete buffer from current context
        if (0 != name)
        {
            glDeleteBuffers(1, &name); // Step 7
            GL_CHECK_ERROR;
//            name = 0;
        }
    }    

}

