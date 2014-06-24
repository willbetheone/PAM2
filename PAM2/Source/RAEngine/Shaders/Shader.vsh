//
//  Shader.vsh
//  qe
//
//  Created by Rinat Abdrashitov on 11/20/2013.
//  Copyright (c) 2013 Rinat Abdrashitov. All rights reserved.
//

attribute vec4 aColor;
attribute vec4 aPosition;
attribute vec3 aNormal;

varying lowp vec4 colorVarying;
varying lowp vec4 vColor;

uniform mat4 uModelViewProjectionMatrix;
uniform mat3 uNormalMatrix;

void main()
{
    vec3 eyeNormal = normalize(uNormalMatrix * aNormal);
    vec3 lightPosition = vec3(0.0, 0.0, 10.0);
    vec4 diffuseColor = vec4(1.0, 1.0, 1.0, 1.0);

    float nDotVP = max(0.0, dot(eyeNormal, normalize(lightPosition)));
    
    colorVarying = diffuseColor * nDotVP;
    vColor = aColor;
    gl_Position = uModelViewProjectionMatrix * aPosition;
}
