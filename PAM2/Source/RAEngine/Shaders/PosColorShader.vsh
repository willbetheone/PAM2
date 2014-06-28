//
//  Shader.vsh
//  qe
//
//  Created by Rinat Abdrashitov on 11/20/2013.
//  Copyright (c) 2013 Rinat Abdrashitov. All rights reserved.
//

attribute vec4 aColor;
attribute vec4 aPosition;

varying lowp vec4 vColor;

uniform mat4 uModelViewProjectionMatrix;

void main()
{
    vColor = aColor;
    gl_Position = uModelViewProjectionMatrix * aPosition;
}
