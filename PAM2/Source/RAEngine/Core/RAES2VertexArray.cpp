//
//  RAES2VertexArray.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-23.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAES2VertexArray.h"
#include <OpenGLES/ES2/glext.h>
#include "RALogManager.h"

namespace RAEngine
{
    RAES2VertexArray::RAES2VertexArray()
    {
        glGenVertexArraysOES(1, &name);
        GL_CHECK_ERROR;
    }
    
    void RAES2VertexArray::bind() const
    {
        glBindVertexArrayOES(name);
        GL_CHECK_ERROR;
    }
    
    void RAES2VertexArray::unbind()
    {
        glBindVertexArrayOES(0);
    }
    
    GLuint RAES2VertexArray::getName() const
    {
        return name;
    }
}