//
//  RAShaderProgram.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAShaderProgram__
#define __PAM2__RAShaderProgram__

#include <iostream>
#include <map>
#include <OpenGLES/ES2/gl.h>

namespace RAEngine
{
    class RAShaderProgram
    {
    public:
        RAShaderProgram();
        int loadProgram(const std::string& vertexShaderPath, const std::string& fragmentShaderPath);
        GLint attributeLocation(const GLchar* name) const noexcept;
        GLint uniformLocation(const GLchar* name) const noexcept;

        static int compileShader(GLuint* shader, GLenum type, const std::string& filename);
        static int linkProgram(GLuint prog);
        static int validateProgram(GLuint prog);

        GLuint program;
    private:
        static std::map<std::string, GLuint> shaderMap;
    };
}

#endif /* defined(__PAM2__RAShaderProgram__) */
