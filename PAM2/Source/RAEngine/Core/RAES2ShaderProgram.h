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
        //TODO comment every function
        
        /*!
         * @brief Create a shader program from vertex and fragment shader files
         */
        int loadProgram(const std::string& vertexShaderPath,
                        const std::string& fragmentShaderPath);
        GLuint getProgram() ;
        GLint getAttributeLocation(const GLchar* name) const;
        GLint getUniformLocation(const GLchar* name) const;
        
    private:
        GLuint program;
        std::string shaderNameKey;
        
        // Store compiled shader programs. vertex+fragment shader name is key, program number is value.
        static std::map<std::string, GLuint> shaderNameToProgramMap;
        // Count how many times program is requested. Delete the program only when count goes to zero.
        static std::map<std::string, int> shaderNameToCountMap;
        
        static GLuint createProgram(const std::string& vertexShaderPath,
                                    const std::string& fragmentShaderPath);
        static int compileShader(GLuint* shader, GLenum type, const std::string& filename);
        static int linkProgram(GLuint prog);
        static int validateProgram(GLuint prog);
        
        ~RAES2ShaderProgram();
    };
}

#endif /* defined(__PAM2__RAES2ShaderProgram__) */
