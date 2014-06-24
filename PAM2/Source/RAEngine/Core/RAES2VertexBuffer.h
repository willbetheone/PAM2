//
//  RAES2VertexBuffer.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAES2VertexBuffer__
#define __PAM2__RAES2VertexBuffer__

#include <iostream>
#import <OpenGLES/ES2/gl.h>

namespace RAEngine
{
    class RAES2VertexBuffer
    {
    public:
        RAES2VertexBuffer(GLsizei aStride,
                          GLsizei count,
                          const GLvoid* dataPtr,
                          GLenum usage,
                          GLenum aTarget);
        ~RAES2VertexBuffer();
        
        GLuint getName() const;
        
        void bufferSubData(GLintptr offset, GLsizeiptr size, const GLvoid* dataPtr);
        
        void enableAttribute(GLuint index) const;
        
        void bind() const;
        
        void prepareToDraw(GLuint index,
                           GLint count,
                           GLsizeiptr offset,
                           GLenum type,
                           GLboolean normalized) const;
        
        static void drawPreparedArrays(GLenum mode,
                                       GLint first,
                                       GLsizei count);
        
        static void drawPreparedArraysIndicies(GLenum mode,
                                               GLenum dataType,
                                               GLsizei numIndcies);
    private:
        GLsizei stride;
        GLsizeiptr bufferSizeBytes;
        GLuint name;
        GLenum target;
    };    
}

#endif /* defined(__PAM2__RAES2VertexBuffer__) */
