//
//  RALogManager.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-17.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RALogManager__
#define __PAM2__RALogManager__

#include <iostream>

/*****LOGGING TO STDOUT AND STDERR*****/
#define SHOULD_LOG 1
#define SHOULD_LOG_INFO 1
#define SHOULD_LOG_ERROR 1
#define SHOULD_LOG_WARN 1
#define SHOULD_LOG_VERBOSE 1
#define SHOULD_LOG_VERY_VERBOSE 1

#if SHOULD_LOG
#define RA_LOG_INFO(Format, ...) \
{ \
    if (SHOULD_LOG_INFO) { \
        fprintf(stdout, "[INFO] [%s at line %i] ", __FUNCTION__, __LINE__); \
        fprintf(stdout, Format, ##__VA_ARGS__); \
        fprintf(stdout, "\n"); \
    }\
} \

#define RA_LOG_ERROR(Format, ...) \
{ \
    if (SHOULD_LOG_ERROR) { \
        fprintf(stderr, "[ERROR] [%s at line %i] ", __FUNCTION__, __LINE__); \
        fprintf(stderr, Format, ##__VA_ARGS__); \
        fprintf(stderr, "\n"); \
    }\
} \

#define RA_LOG_WARN(Format, ...) \
{ \
    if (SHOULD_LOG_WARN) { \
        fprintf(stderr, "[WARNING] [%s at line %i] ", __FUNCTION__, __LINE__); \
        fprintf(stderr, Format, ##__VA_ARGS__); \
        fprintf(stderr, "\n"); \
    }\
} \

#define RA_LOG_VERBOSE(Format, ...) \
{ \
    if (SHOULD_LOG_VERBOSE) { \
        fprintf(stderr, "[VERBOSE] [%s at line %i] ", __FUNCTION__, __LINE__); \
        fprintf(stderr, Format, ##__VA_ARGS__); \
        fprintf(stderr, "\n"); \
    }\
} \

#define RA_LOG_VERY_VERBOSE(Format, ...) \
{ \
    if (SHOULD_LOG_VERY_VERBOSE) { \
        fprintf(stderr, "[VERY_VERBOSE] [%s at line %i] ", __FUNCTION__, __LINE__); \
        fprintf(stderr, Format, ##__VA_ARGS__); \
        fprintf(stderr, "\n"); \
    }\
} \

#else
#define RA_LOG_INFO(Format, ...) {}
#define RA_LOG_ERROR(Format, ...) {}
#define RA_LOG_WARN(Format, ...) {}
#endif

/*****OPENGL LOGGING*****/
#if DEBUG //Xcode specific DEBUG macro
#define ENABLE_GL_CHECK 1
#else
#define ENABLE_GL_CHECK 0
#endif

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
fprintf(stderr,"[ERROR] OpenGL ES2 error 0x%04X %s in %s at line %i\n in file:%s", e, errorString, __FUNCTION__, __LINE__, __FILE__); \
} \
}
#else
#define GL_CHECK_ERROR {}
#endif


#endif /* defined(__PAM2__RALogManager__) */
