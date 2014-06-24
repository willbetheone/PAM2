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
#import <OpenGLES/ES2/gl.h>

namespace RAEngine
{
    class RAES2ShaderProgram
    {
    public:        
        
        ~RAES2ShaderProgram();
        
        /**
         Creates shader program. Compiles vertex/fragment shaders and attaches them
         to the shader program.
         @param vertexShaderPath Absolute path to a vertex shader source
         @param fragmentShaderPath Absolute path to a fragment shader source
         @return 0 is returned upon failure
         */
        int loadProgram(const std::string& vertexShaderPath, 
                        const std::string& fragmentShaderPath);
        
        /**
         Get compiled shader program
         @return shader program
         */
        GLuint getProgram() const;
        
        /**
         Get the location of an attribute variable
         @param name name of the attribute variable whose location is to be queried.
         @return location of attribute variable
         */
        GLint getAttributeLocation(const GLchar* name) const;
        
        /**
         Get the location of an uniform variable
         @param name name of the uniform variable whose location is to be queried.
         @return location of uniform variable
         */
        GLint getUniformLocation(const GLchar* name) const;
        
    private:
        GLuint program; // shader program
        std::string shaderNameKey; // vertex+fragment shader name
        
        // Store compiled shader programs. vertex+fragment shader name is key, program number is value.
        static std::map<std::string, GLuint> shaderNameToProgramMap;
        // Count how many times program is requested. Delete the program only when count goes to zero.
        static std::map<std::string, int> shaderNameToCountMap;
        
        static GLuint createProgram(const std::string& vertexShaderPath,
                                    const std::string& fragmentShaderPath);
        
        static int compileShader(GLuint* shader, GLenum type, const std::string& filename);
        static int linkProgram(GLuint prog);
        static int validateProgram(GLuint prog);
    };
}

#endif /* defined(__PAM2__RAES2ShaderProgram__) */
