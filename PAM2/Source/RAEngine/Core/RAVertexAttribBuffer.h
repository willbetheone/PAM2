//
//  RAVertexAttribBuffer.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAVertexAttribBuffer__
#define __PAM2__RAVertexAttribBuffer__

#include <iostream>
#include <OpenGLES/ES2/gl.h>
#include <assert.h> 

namespace RAEngine {
    
    class RAVertexAttribBuffer {
    public:
        RAVertexAttribBuffer(GLsizei aStride,
                             GLsizei count,
                             const GLvoid* dataPtr,
                             GLenum usage,
                             GLenum aTarget);
        ~RAVertexAttribBuffer();
        
        void bufferSubData(GLintptr offset, GLsizeiptr size, const GLvoid* dataPtr);
        void enableAttribute(GLuint index);
        void bind();
        void prepareToDraw(GLuint index,
                           GLint count,
                           GLsizeiptr offset,
                           GLenum type,
                           GLboolean normalized);
        void drawPreparedArrays(GLenum mode,
                                GLint first,
                                GLsizei count);
        void drawPreparedArraysIndicies(GLenum mode,
                                        GLenum dataType,
                                        GLsizei numIndcies);
    private:
        GLsizei stride;
        GLsizeiptr bufferSizeBytes;
        GLuint name;
        GLenum target;
    };    
}

#endif /* defined(__PAM2__RAVertexAttribBuffer__) */
