//
//  Shader.fsh
//  qe
//
//  Created by Rinat Abdrashitov on 11/20/2013.
//  Copyright (c) 2013 Rinat Abdrashitov. All rights reserved.
//

varying lowp vec4 vColor;

void main()
{
    gl_FragColor = vColor;
}
