//
//  RABoundingBox.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-23.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RABoundingBox.h"

namespace RAEngine {
    
    using namespace CGLA;
    using namespace std;
    
    RABoundingBox::RABoundingBox() : RABoundingBox(Vec3f(-0.5,-0.5,-0.5), Vec3f(0.5,0.5,0.5))
    {
    };

    RABoundingBox::RABoundingBox(CGLA::Vec3f pmin, CGLA::Vec3f pmax)
    {
        minBound = pmin;
        maxBound = pmax;
        
        Vec3f mid = maxBound - minBound;
        radius = 0.5*mid.length();
        center =  minBound + radius*normalize(mid);
        
        width = fabsf(maxBound[0] - minBound[0]);
        height = fabsf(maxBound[1] - minBound[1]);
        depth = fabsf(maxBound[2] - minBound[2]);
    }
    
    Bounds RABoundingBox::getBoundingBox() const
    {
        Bounds bounds = {minBound, maxBound, center, radius};
        return bounds;
    }
    
    void RABoundingBox::getVertexData(CGLA::Vec3f*& vertexPositions,
                                      CGLA::Vec4uc*& vertexColors,
                                      std::vector<unsigned int>*& indicies)
    {
        Vec4uc color(250,0,0,255);
        vertexPositions = new Vec3f[8]; 
        vertexColors = new Vec4uc[8];
        fill_n(vertexColors, 8, color);

        Vec3f xAxis = Vec3f(width,0,0);
        Vec3f zAxis = Vec3f(0,0,depth);
        
        vertexPositions[0] = minBound;
        vertexPositions[1] = minBound + xAxis;
        vertexPositions[2] = minBound + xAxis + zAxis;
        vertexPositions[3] = minBound + zAxis;

        vertexPositions[4] = maxBound;
        vertexPositions[5] = maxBound - xAxis;
        vertexPositions[6] = maxBound - xAxis - zAxis;
        vertexPositions[7] = maxBound - zAxis;

        indicies = new vector<unsigned int>({0,1,1,2,2,3,3,0, //bottom face
                                             4,5,5,6,6,7,7,4, //top face
                                             0,6,4,2,3,5,1,7  //sides
        });
    }
    
    void RABoundingBox::setupShaders(const std::string vertexShader, const std::string fragmentShader)
    {
        
        drawShaderProgram = new RAES2ShaderProgram();
        drawShaderProgram->loadProgram(vertexShader, fragmentShader);
        
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("aPosition");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("aColor");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
    }
    
    void RABoundingBox::bufferVertexDataToGPU()
    {
        Vec3f* vertexPositions;
        Vec4uc* vertexColors;
        std::vector<unsigned int>* indicies;
        getVertexData(vertexPositions, vertexColors, indicies);
        
        numVerticies = 8;
        positionDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                   numVerticies,
                                                   vertexPositions,
                                                   GL_STATIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(attrib[ATTRIB_POSITION]);
        
        colorDataBuffer = new RAES2VertexBuffer(sizeof(Vec4uc),
                                                numVerticies,
                                                vertexColors,
                                                GL_STATIC_DRAW,
                                                GL_ARRAY_BUFFER);
        colorDataBuffer->enableAttribute(attrib[ATTRIB_COLOR]);

        numIndicies = indicies->size();
        indexDataBuffer = new RAES2VertexBuffer(sizeof(unsigned int),
                                                numIndicies,
                                                indicies->data(),
                                                GL_STATIC_DRAW,
                                                GL_ELEMENT_ARRAY_BUFFER);
        delete[] vertexPositions;
        delete[] vertexColors;
        delete indicies;
    }
    
    void RABoundingBox::draw() const
    {
        if (!enabled) {
            return;
        }
        
        Mat4x4 mvpMat = transpose(getModelViewProjectionMatrix());
        
        glPushGroupMarkerEXT(0, "Drawing Bounding Box");
        
        glUseProgram(drawShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;

        glLineWidth(2.0f);
        
        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attrib[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        
        colorDataBuffer->bind();
        colorDataBuffer->prepareToDraw(attrib[ATTRIB_COLOR], 4, 0, GL_UNSIGNED_BYTE, GL_TRUE);
        
        indexDataBuffer->bind();
        RAES2VertexBuffer::drawPreparedArraysIndicies(GL_LINES, GL_UNSIGNED_INT, numIndicies);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();
    }
}