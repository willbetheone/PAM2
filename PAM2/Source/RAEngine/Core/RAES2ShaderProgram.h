//
//  RAES2ShaderProgram.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__RAES2ShaderProgram__
#define __PAM2__RAES2ShaderProgram__

#include <iostream>
#include <map>
#include <OpenGLES/ES2/gl.h>

namespace RAEngine
{
    class RAES2ShaderProgram
    {
    public:
        
        int loadProgram(const std::string& vertexShaderPath, const std::string& fragmentShaderPath);
        
        GLuint getProgram() const noexcept;
        GLint getAttributeLocation(const GLchar* name) const noexcept;
        GLint getUniformLocation(const GLchar* name) const noexcept;

        static GLuint createProgram(const std::string& vertexShaderPath,
                                    const std::string& fragmentShaderPath);
    private:
        GLuint program;
        std::string shaderNameKey;
        
        static std::map<std::string, GLuint> shaderNameToProgramMap;
        static std::map<std::string, int> shaderNameToCountMap;
        
        static int compileShader(GLuint* shader, GLenum type, const std::string& filename) noexcept;
        static int linkProgram(GLuint prog) noexcept;
        static int validateProgram(GLuint prog) noexcept;
        
        ~RAES2ShaderProgram();
    };
}

#endif /* defined(__PAM2__RAES2ShaderProgram__) */
