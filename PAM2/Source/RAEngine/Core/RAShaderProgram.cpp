//
//  RAShaderProgram.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAShaderProgram.h"
#include <fstream>
#include <iostream>

namespace RAEngine {
    using namespace std;
    
    map<string, GLuint> RAShaderProgram::shaderMap{};
    
    void checkError2() {
        GLenum error = glGetError();
        if (GL_NO_ERROR != error) {
            printf("GL Error: 0x%x", error);
        }
    }
    
    //Default constructor
    RAShaderProgram::RAShaderProgram() : program{0} {};
    
    //Create a shader program from vertex and fragment shader files
    int RAShaderProgram::loadProgram(const string& vertexShaderPath, const string& fragmentShaderPath)
    {
        
        //TODO make this static
        string key = vertexShaderPath+fragmentShaderPath;
        map<string, GLuint>::iterator it = shaderMap.find(key);
        if (it != shaderMap.end()) {
            program = shaderMap[key];
        } else {
            GLuint vertShader, fragShader;
            
            // Create shader program.
            program = glCreateProgram();
            checkError2();
            
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
            checkError2();
            glAttachShader(program, fragShader);
            checkError2();
            
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
            checkError2();
            
            // Release vertex and fragment shaders.
            if (vertShader) {
                glDetachShader(program, vertShader);
                checkError2();
                glDeleteShader(vertShader);
                checkError2();
            }
            if (fragShader) {
                glDetachShader(program, fragShader);
                checkError2();
                glDeleteShader(fragShader);
                checkError2();
            }
            

            shaderMap[key] = program;
        }

        return 1;
    }
    
    GLint RAShaderProgram::attributeLocation(const GLchar* name) const noexcept
    {
        return glGetAttribLocation(program, name);
    }
    
    GLint RAShaderProgram::uniformLocation(const GLchar* name) const noexcept
    {
        return glGetUniformLocation(program, name);
    }


#pragma mark - SHADER COMPILATION

    int RAShaderProgram::compileShader(GLuint* shader, GLenum type, const string& filename)
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
        checkError2();
        glShaderSource(*shader, 1, &source, NULL);
        checkError2();
        glCompileShader(*shader);
        checkError2();
        
        GLint logLength;
        glGetShaderiv(*shader, GL_INFO_LOG_LENGTH, &logLength);
        checkError2();
        if (logLength > 0) {
            GLchar *log = (GLchar *)malloc(logLength);
            glGetShaderInfoLog(*shader, logLength, &logLength, log);
            cerr << "Shader compile log:\n " << log;
            free(log);
        }

        
        glGetShaderiv(*shader, GL_COMPILE_STATUS, &status);
        checkError2();
        if (status == 0) {
            glDeleteShader(*shader);
            cerr << "Couldnt compile shader. Deleting it...\n ";
            return 0;
        }
        
        return 1;
    }

    int RAShaderProgram::linkProgram(GLuint prog)
    {
        GLint status;
        glLinkProgram(prog);
        checkError2();
        
        GLint logLength;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLength);
        checkError2();
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

    int RAShaderProgram::validateProgram(GLuint prog)
    {
        GLint logLength, status;
        
        glValidateProgram(prog);
        checkError2();
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