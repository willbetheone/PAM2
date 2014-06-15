//
//  TempMesh.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-12.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "TempMesh.h"

namespace TempToDelete {
    GLfloat gCubeVertexData[216] =
    {
        // Data layout for each line below is:
        // positionX, positionY, positionZ,     normalX, normalY, normalZ,
        0.5f, -0.5f, -0.5f,        1.0f, 0.0f, 0.0f,
        0.5f, 0.5f, -0.5f,         1.0f, 0.0f, 0.0f,
        0.5f, -0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
        0.5f, -0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
        0.5f, 0.5f, -0.5f,          1.0f, 0.0f, 0.0f,
        0.5f, 0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
        
        0.5f, 0.5f, -0.5f,         0.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, -0.5f,        0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.5f,          0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.5f,          0.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, -0.5f,        0.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, 0.5f,         0.0f, 1.0f, 0.0f,
        
        -0.5f, 0.5f, -0.5f,        -1.0f, 0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,       -1.0f, 0.0f, 0.0f,
        -0.5f, 0.5f, 0.5f,         -1.0f, 0.0f, 0.0f,
        -0.5f, 0.5f, 0.5f,         -1.0f, 0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,       -1.0f, 0.0f, 0.0f,
        -0.5f, -0.5f, 0.5f,        -1.0f, 0.0f, 0.0f,
        
        -0.5f, -0.5f, -0.5f,       0.0f, -1.0f, 0.0f,
        0.5f, -0.5f, -0.5f,        0.0f, -1.0f, 0.0f,
        -0.5f, -0.5f, 0.5f,        0.0f, -1.0f, 0.0f,
        -0.5f, -0.5f, 0.5f,        0.0f, -1.0f, 0.0f,
        0.5f, -0.5f, -0.5f,        0.0f, -1.0f, 0.0f,
        0.5f, -0.5f, 0.5f,         0.0f, -1.0f, 0.0f,
        
        0.5f, 0.5f, 0.5f,          0.0f, 0.0f, 1.0f,
        -0.5f, 0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
        -0.5f, 0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, 0.5f,        0.0f, 0.0f, 1.0f,
        
        0.5f, -0.5f, -0.5f,        0.0f, 0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,       0.0f, 0.0f, -1.0f,
        0.5f, 0.5f, -0.5f,         0.0f, 0.0f, -1.0f,
        0.5f, 0.5f, -0.5f,         0.0f, 0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,       0.0f, 0.0f, -1.0f,
        -0.5f, 0.5f, -0.5f,        0.0f, 0.0f, -1.0f
    };

    
    using namespace RAEngine;
    using namespace std;
    
    int VERTEX_SIZE =  3 * sizeof(GLfloat);
    int COLOR_SIZE =  4 * sizeof(unsigned char);
    int INDEX_SIZE  = sizeof(unsigned int);
    
    TempMesh::TempMesh() : RAMesh()
    {
    }
    
    void TempMesh::setShaders(const string& vShader, const string& fShader)
    {
        drawShaderProgram = new RAES2ShaderProgram();
        
        drawShaderProgram->loadProgram(vShader, fShader);
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("position");
        attrib[ATTRIB_NORMAL] = drawShaderProgram->getAttributeLocation("normal");
//        attrib[ATTRIB_COLOR] = drawShaderProgram->attributeLocation("color");
  
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("modelViewProjectionMatrix");
        uniforms[UNIFORM_NORMAL_MATRIX] = drawShaderProgram->getUniformLocation("normalMatrix");
    }
    
    void TempMesh::setVertexData()
    {
        numVerticies = sizeof(gCubeVertexData)/(2*VERTEX_SIZE);
        vertexDataBuffer = new RAVertexAttribBuffer(2*VERTEX_SIZE,
                                                    numVerticies, 
                                                    gCubeVertexData,
                                                    GL_STATIC_DRAW,
                                                    GL_ARRAY_BUFFER);
        vertexDataBuffer->enableAttribute(ATTRIB_POSITION);
        vertexDataBuffer->enableAttribute(ATTRIB_NORMAL);
        
    }
    
   
    void TempMesh::draw()
    {
        //added a new line
        Mat3x3 nMat = getNormalMatrix();
        Mat4x4 mvpMat = getModelViewProjectionMatrix();
        
        glUseProgram(drawShaderProgram->getProgram());
        glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        glUniformMatrix3fv(uniforms[UNIFORM_NORMAL_MATRIX], 1, 0, nMat.get());
        vertexDataBuffer->bind();
        vertexDataBuffer->prepareToDraw(attrib[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        vertexDataBuffer->prepareToDraw(attrib[ATTRIB_NORMAL], 3, VERTEX_SIZE, GL_FLOAT, GL_FALSE);
        vertexDataBuffer->drawPreparedArrays(GL_TRIANGLES, 0, numVerticies);

    }
}
