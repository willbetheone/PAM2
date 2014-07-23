//
//  RAES2DepthBuffer.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-07.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAES2OffScreenBuffer.h"
#include <OpenGLES/ES2/glext.h>
#include "RALogManager.h"

namespace RAEngine {
    
    RAES2OffScreenBuffer::RAES2OffScreenBuffer(){ }
    
    RAES2OffScreenBuffer::~RAES2OffScreenBuffer()
    {
        glDeleteRenderbuffers(1, &colorBuffer);
        glDeleteRenderbuffers(1, &depthBuffer);
        glDeleteFramebuffers(1, &frameBuffer);
    }
    
    void RAES2OffScreenBuffer::createOffScreenBuffer(GLsizei width, GLsizei height)
    {
        //Create additional Buffers
        //Create framebuffer and attach color/depth renderbuffers
        glGenFramebuffers(1, &frameBuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);
        
        glGenRenderbuffers(1, &colorBuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, colorBuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorBuffer);
        
        glGenRenderbuffers(1, &depthBuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer);
        
        int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            RA_LOG_ERROR("Couldnt create offscreen buffer");
        }
        
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    
    void RAES2OffScreenBuffer::bind()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);
    }
    
    void RAES2OffScreenBuffer::unbind()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}