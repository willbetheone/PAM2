//
//  RAVertexAttribBuffer.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAVertexAttribBuffer.h"

namespace RAEngine {
    
    void checkError();
    
    RAVertexAttribBuffer::RAVertexAttribBuffer(GLsizei aStride,
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
        checkError();
        glBindBuffer(target, name);
        checkError();
        glBufferData(target,  // Initialize buffer contents
                     bufferSizeBytes,  // Number of bytes to copy
                     dataPtr,          // Address of bytes to copy
                     usage);           // Hint: cache in GPU memory
        checkError();
        assert(0 != name);

        
    }
    
    
    void RAVertexAttribBuffer::bufferSubData(GLintptr offset,
                                                    GLsizeiptr size,
                                                    const GLvoid* dataPtr)
    {
        assert(offset + size <= bufferSizeBytes);
        glBufferSubData(target, 0, bufferSizeBytes, dataPtr);
    }

    void RAVertexAttribBuffer::enableAttribute(GLuint index) {
        glEnableVertexAttribArray(index);
        checkError();
    }
    
    void RAVertexAttribBuffer::bind() {
        glBindBuffer(target, name);
        checkError();
    }
    
    void RAVertexAttribBuffer::prepareToDraw(GLuint index,
                                             GLint count,
                                             GLsizeiptr offset,
                                             GLenum type,
                                             GLboolean normalized)
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
        
        checkError();
    }
    
    void RAVertexAttribBuffer::drawPreparedArrays(GLenum mode,
                                                  GLint first,
                                                  GLsizei count)
    {
        glDrawArrays(mode, first, count);
    }
    
    void RAVertexAttribBuffer::drawPreparedArraysIndicies(GLenum mode,
                                                          GLenum dataType,
                                                          GLsizei numIndcies)
    {
        glDrawElements(mode, numIndcies, dataType, 0);
    }


    RAVertexAttribBuffer::~RAVertexAttribBuffer()
    {
        // Delete buffer from current context
        if (0 != name)
        {
            glDeleteBuffers (1, &name); // Step 7
            name = 0;
        }
    }    
    
    void checkError() {
        GLenum error = glGetError();
        if (GL_NO_ERROR != error) {
            printf("GL Error: 0x%x\n", error);
        }
    }
}

