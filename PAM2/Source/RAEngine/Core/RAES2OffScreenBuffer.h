//
//  RAES2OffScreenBuffer.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-07.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAES2OffScreenBuffer__
#define __PAM2__RAES2OffScreenBuffer__

#include <iostream>
#include <OpenGLES/ES2/gl.h>

namespace RAEngine {

    class RAES2OffScreenBuffer
    {
    public:
        RAES2OffScreenBuffer();
        ~RAES2OffScreenBuffer();
        void createOffScreenBuffer(GLsizei width, GLsizei height);
        void bind();
        void unbind();
    private:
        GLuint frameBuffer;
        GLuint colorBuffer;
        GLuint depthBuffer;
    };
}

#endif /* defined(__PAM2__RAES2OffScreenBuffer__) */
