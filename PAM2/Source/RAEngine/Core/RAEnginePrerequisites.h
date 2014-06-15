//
//  RAEnginePrerequisites.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-14.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef PAM2_RAEnginePrerequisites_h
#define PAM2_RAEnginePrerequisites_h

#define ENABLE_GL_CHECK 1
#if ENABLE_GL_CHECK
#define GL_CHECK_ERROR \
{ \
    GLenum e = glGetError(); \
    if (e != GL_NO_ERROR) \
    { \
        const char * errorString = ""; \
        switch(e) \
        { \
            case GL_INVALID_ENUM:       errorString = "GL_INVALID_ENUM";        break; \
            case GL_INVALID_VALUE:      errorString = "GL_INVALID_VALUE";       break; \
            case GL_INVALID_OPERATION:  errorString = "GL_INVALID_OPERATION";   break; \
            case GL_OUT_OF_MEMORY:      errorString = "GL_OUT_OF_MEMORY";       break; \
            default:                                                            break; \
        } \
        printf("OpenGL ES2 error 0x%04X %s in %s at line %i\n in file:%s", e, errorString, __PRETTY_FUNCTION__, __LINE__, __FILE__); \
    } \
}
#else
#define GL_CHECK_ERROR {}
#endif

#endif
