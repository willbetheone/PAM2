//
//  RAPolyLine.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "RAPolyLine.h"

namespace RAEngine
{
    using namespace CGLA;
    using namespace std;
    
    void RAPolyLine::setupShaders(const std::string vertexShader, const std::string fragmentShader)
    {
        
        drawShaderProgram = new RAES2ShaderProgram();
        drawShaderProgram->loadProgram(vertexShader, fragmentShader);
        
        attrib[ATTRIB_POSITION] = drawShaderProgram->getAttributeLocation("aPosition");
        attrib[ATTRIB_COLOR] = drawShaderProgram->getAttributeLocation("aColor");
        
        uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX] = drawShaderProgram->getUniformLocation("uModelViewProjectionMatrix");
    }
    
    void RAPolyLine::bufferVertexDataToGPU(std::vector<CGLA::Vec3f> points, CGLA::Vec4uc color)
    {
        numVerticies = points.size();
        positionDataBuffer = new RAES2VertexBuffer(sizeof(Vec3f),
                                                   numVerticies,
                                                   points.data(),
                                                   GL_DYNAMIC_DRAW,
                                                   GL_ARRAY_BUFFER);
        positionDataBuffer->enableAttribute(attrib[ATTRIB_POSITION]);
        
        Vec4uc* vertexColors = new Vec4uc[points.size()];
        fill_n(vertexColors, points.size(), color);
        for (int i = 0; i < points.size(); i++) {
            if (i%2 == 0) {
                vertexColors[i] = Vec4uc(0,0,0,255);
            }
        }
        
        colorDataBuffer = new RAES2VertexBuffer(sizeof(Vec4uc),
                                                numVerticies,
                                                vertexColors,
                                                GL_DYNAMIC_DRAW,
                                                GL_ARRAY_BUFFER);
        colorDataBuffer->enableAttribute(attrib[ATTRIB_COLOR]);
        
        delete[] vertexColors;
    }
    
    Bounds RAPolyLine::getBoundingBox() const
    {
        return Bounds{};
    }
    
    void RAPolyLine::draw() const
    {
        if(!enabled)
            return;
            
        Mat4x4 mvpMat = transpose(getModelViewProjectionMatrix());
        
        glPushGroupMarkerEXT(0, "Drawing a Line");
        
        glUseProgram(drawShaderProgram->getProgram());
        GL_CHECK_ERROR;
        glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEWPROJECTION_MATRIX], 1, 0, mvpMat.get());
        GL_CHECK_ERROR;
        
        glLineWidth(5.0f);
        
        positionDataBuffer->bind();
        positionDataBuffer->prepareToDraw(attrib[ATTRIB_POSITION], 3, 0, GL_FLOAT, GL_FALSE);
        
        colorDataBuffer->bind();
        colorDataBuffer->prepareToDraw(attrib[ATTRIB_COLOR], 4, 0, GL_UNSIGNED_BYTE, GL_TRUE);

        RAES2VertexBuffer::drawPreparedArrays(GL_LINE_STRIP, 0, numVerticies);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        
        glPopGroupMarkerEXT();

    }
}
