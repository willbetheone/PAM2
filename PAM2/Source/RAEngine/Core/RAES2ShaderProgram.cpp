//
//  RAES2ShaderProgram.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAES2ShaderProgram.h"
#include <fstream>
#include <iostream>
#include <OpenGLES/ES2/glext.h>
#include "RALogManager.h"
#include "assert.h"

namespace RAEngine
{   
    using namespace std;    

    map<string, GLuint> RAES2ShaderProgram::shaderNameToProgramMap{};
    map<string, int> RAES2ShaderProgram::shaderNameToCountMap{};

#pragma mark - CONSTRUCTOR/DESTRUCTOR
    
    RAES2ShaderProgram::~RAES2ShaderProgram()
    {
        map<string, int>::iterator it = shaderNameToCountMap.find(shaderNameKey);
        if (it != shaderNameToCountMap.end()) {
            int newCount = it->second - 1;
            if (newCount == 0) {
                glDeleteProgram(program);
                GL_CHECK_ERROR;
                shaderNameToProgramMap.erase(shaderNameKey);
                shaderNameToCountMap.erase(shaderNameKey);
            } else {
                shaderNameToCountMap[shaderNameKey] = newCount;
            }
        }
    }

#pragma mark - PUBLIC METHODS
    
    int RAES2ShaderProgram::loadProgram(const std::string& vertexShaderPath,
                                        const std::string& fragmentShaderPath)
    {
        shaderNameKey = vertexShaderPath + fragmentShaderPath;
        map<string, GLuint>::iterator it = shaderNameToProgramMap.find(shaderNameKey);
        
        //check if program was previously compiled and return it, create new otherwise
        if (it != shaderNameToProgramMap.end()) {
            program = it->second;
            shaderNameToCountMap[shaderNameKey] += 1;
        } else {
            program = createProgram(vertexShaderPath, fragmentShaderPath);
            if (program == 0) {
                return 0;
            }
            
            shaderNameToCountMap[shaderNameKey] = 1;
            shaderNameToProgramMap[shaderNameKey] = program;
        }

        return 1;
    }
    
    GLuint RAES2ShaderProgram::getProgram() const
    {
        return program;
    }
    
    GLint RAES2ShaderProgram::getAttributeLocation(const GLchar* name) const
    {
        GLint loc = glGetAttribLocation(program, name);
        GL_CHECK_ERROR;
        assert(loc != -1);
        return loc;
    }
    
    GLint RAES2ShaderProgram::getUniformLocation(const GLchar* name) const
    {
        GLint loc = glGetUniformLocation(program, name);
        GL_CHECK_ERROR;
        assert(loc != -1);
        return loc;
        
    }

#pragma mark - PRIVATE METHODS
    
    GLuint RAES2ShaderProgram::createProgram(const std::string& vertexShaderPath,
                                             const std::string& fragmentShaderPath)
    {
        GLuint vertShader, fragShader;
        
        // Create shader program.
        GLuint program = glCreateProgram();
        if (program == 0) {
            return 0;
        }
        
        // Create and compile vertex/fragment shaders
        if (!compileShader(&vertShader, GL_VERTEX_SHADER, vertexShaderPath)) {
            RA_LOG_ERROR("Failed to compile vertex shader");
            return 0;
        }
        
        if (!compileShader(&fragShader, GL_FRAGMENT_SHADER, fragmentShaderPath)) {
            RA_LOG_ERROR("Failed to compile fragmet shader");
            return 0;
        }
        
        // Attach vertex/fragment shaders to program
        glAttachShader(program, vertShader);
        GL_CHECK_ERROR;
        glAttachShader(program, fragShader);
        GL_CHECK_ERROR;
        
        // Link program
        if (!linkProgram(program)) {
            RA_LOG_ERROR("Failed to link program %i", program);
            if (vertShader) {
                glDeleteShader(vertShader);
                vertShader = 0;
            }
            if (fragShader) {
                glDeleteShader(fragShader);
                fragShader = 0;
            }
            if (program) {
                glDeleteProgram(program);
                program = 0;
            }
            return 0;
        }
        GL_CHECK_ERROR;
        
        // Release vertex and fragment shaders.
        if (vertShader) {
            glDetachShader(program, vertShader);
            GL_CHECK_ERROR;
            glDeleteShader(vertShader);
            GL_CHECK_ERROR;
        }
        if (fragShader) {
            glDetachShader(program, fragShader);
            GL_CHECK_ERROR;
            glDeleteShader(fragShader);
            GL_CHECK_ERROR;
        }
        
        return program;
    }

    int RAES2ShaderProgram::compileShader(GLuint* shader, GLenum type, const string& filename)
    {
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            RA_LOG_ERROR("Couldnt open shader file %s", filename.c_str());
            return 0;
        }
        std::string content((std::istreambuf_iterator<GLchar>(ifs)),
                            (std::istreambuf_iterator<GLchar>()));

        const GLchar* source = content.c_str();
        if (source == nullptr) {
            RA_LOG_ERROR("Couldnt load shader text %s", filename.c_str());
            return 0;
        }
        
        *shader = glCreateShader(type);
        GL_CHECK_ERROR;
        glShaderSource(*shader, 1, &source, NULL);
        GL_CHECK_ERROR;
        glCompileShader(*shader);
        GL_CHECK_ERROR;
        
#ifdef DEBUG
#if DEBUG
        GLint logLength;
        glGetShaderiv(*shader, GL_INFO_LOG_LENGTH, &logLength);
        GL_CHECK_ERROR;
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetShaderInfoLog(*shader, logLength, &logLength, log);
            RA_LOG_WARN("Shader compile log: %s", log);
            free(log);
        }
#endif
#endif

        GLint status;
        glGetShaderiv(*shader, GL_COMPILE_STATUS, &status);
        GL_CHECK_ERROR;
        if (status == 0) {
            glDeleteShader(*shader);
            RA_LOG_ERROR("Couldnt compile shader. Deleting it...");
            return 0;
        }
        
        return 1;
    }


    int RAES2ShaderProgram::linkProgram(GLuint prog)
    {
        GLint status;
        glLinkProgram(prog);
        GL_CHECK_ERROR;

#ifdef DEBUG
#if DEBUG
        GLint logLength;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLength);
        GL_CHECK_ERROR;
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetProgramInfoLog(prog, logLength, &logLength, log);
            RA_LOG_WARN("Program link log: %s", log);
            free(log);
        }
#endif
#endif
        glGetProgramiv(prog, GL_LINK_STATUS, &status);
        if (status == 0) {
            RA_LOG_ERROR("Couldnt link");
            return 0;
        }
        
        return 1;
    }

    int RAES2ShaderProgram::validateProgram(GLuint prog) 
    {
        GLint logLength, status;
        
        glValidateProgram(prog);
        GL_CHECK_ERROR;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLength);
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetProgramInfoLog(prog, logLength, &logLength, log);
            RA_LOG_ERROR("Program validate log: %s", log);
            free(log);
        }
        
        glGetProgramiv(prog, GL_VALIDATE_STATUS, &status);
        if (status == 0) {
            RA_LOG_ERROR("Couldn't validate");
            return 0;
        }
        
        return 1;
    }
}