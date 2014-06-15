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
#include "RAEnginePrerequisites.h"

// GL_CHECK_ERROR should be defined in some included header and handle OpenGL error checking
#ifndef GL_CHECK_ERROR
#define GL_CHECK_ERROR {}
#endif

namespace RAEngine {
    
    using namespace std;
    
    // Store compiled shader programs. Use vertex + fragment shader
    map<string, GLuint> RAES2ShaderProgram::shaderNameToProgramMap{};
    // Count how many times program is requested. Delete the program only when count goes to zero.
    map<string, int> RAES2ShaderProgram::shaderNameToCountMap{};
    
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
    
    //Create a shader program from vertex and fragment shader files
    int RAES2ShaderProgram::loadProgram(const string& vertexShaderPath,
                                        const string& fragmentShaderPath)
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
    
    GLuint RAES2ShaderProgram::getProgram() const noexcept
    {
        return program;
    }
    
    GLint RAES2ShaderProgram::getAttributeLocation(const GLchar* name) const noexcept
    {
        return glGetAttribLocation(program, name);
    }
    
    GLint RAES2ShaderProgram::getUniformLocation(const GLchar* name) const noexcept
    {
        return glGetUniformLocation(program, name);
    }

#pragma mark - SHADER COMPILATION
    
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
            cerr << "Failed to compile vertex shader";
            return 0;
        }
        
        if (!compileShader(&fragShader, GL_FRAGMENT_SHADER, fragmentShaderPath)) {
            cerr << "Failed to compile fragmet shader";
            return 0;
        }
        
        // Attach vertex/fragment shaders to program
        glAttachShader(program, vertShader);
        GL_CHECK_ERROR;
        glAttachShader(program, fragShader);
        GL_CHECK_ERROR;
        
        // Link program
        if (!linkProgram(program)) {
            cerr << "Failed to link program" << program;
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


    int RAES2ShaderProgram::compileShader(GLuint* shader, GLenum type, const string& filename) noexcept
    {
        GLint status;
        
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            cerr << "Couldnt open shader file " << filename;
            return 0;
        }
        std::string content((std::istreambuf_iterator<GLchar>(ifs)),
                            (std::istreambuf_iterator<GLchar>()));
        const GLchar* source = content.c_str();
        
        if (source == nullptr) {
            cerr << "Couldnt load shader text " << filename;
            return 0;
        }
        
        *shader = glCreateShader(type);
        GL_CHECK_ERROR;
        glShaderSource(*shader, 1, &source, NULL);
        GL_CHECK_ERROR;
        glCompileShader(*shader);
        GL_CHECK_ERROR;
        
        GLint logLength;
        glGetShaderiv(*shader, GL_INFO_LOG_LENGTH, &logLength);
        GL_CHECK_ERROR;
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetShaderInfoLog(*shader, logLength, &logLength, log);
            cerr << "Shader compile log:\n " << log;
            free(log);
        }

        
        glGetShaderiv(*shader, GL_COMPILE_STATUS, &status);
        GL_CHECK_ERROR;
        if (status == 0) {
            glDeleteShader(*shader);
            cerr << "Couldnt compile shader. Deleting it...\n ";
            return 0;
        }
        
        return 1;
    }

    int RAES2ShaderProgram::linkProgram(GLuint prog) noexcept
    {
        GLint status;
        glLinkProgram(prog);
        GL_CHECK_ERROR;
        
        GLint logLength;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLength);
        GL_CHECK_ERROR;
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetProgramInfoLog(prog, logLength, &logLength, log);
            cerr << "Program link log:\n" << log;
            free(log);
        }
        
        glGetProgramiv(prog, GL_LINK_STATUS, &status);
        if (status == 0) {
            cerr << "Couldn't link:\n";
            return 0;
        }
        
        return 1;
    }

    int RAES2ShaderProgram::validateProgram(GLuint prog) noexcept
    {
        GLint logLength, status;
        
        glValidateProgram(prog);
        GL_CHECK_ERROR;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLength);
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetProgramInfoLog(prog, logLength, &logLength, log);
            cerr << "Program validate log:\n" << log;
            free(log);
        }
        
        glGetProgramiv(prog, GL_VALIDATE_STATUS, &status);
        if (status == 0) {
            cerr << "Couldn't validate:\n";
            return 0;
        }
        
        return 1;
    }
}