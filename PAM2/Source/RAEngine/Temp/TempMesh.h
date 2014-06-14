//
//  TempMesh.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-12.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__TempMesh__
#define __PAM2__TempMesh__

#include <iostream>
#include "RAMesh.h"

namespace TempToDelete
{
    class TempMesh : public RAEngine::RAMesh
    {
    public:
        TempMesh();
        void setShaders(const std::string& vShader, const std::string& fShader);
        void setVertexData();
        void draw();
    };
    
}


#endif /* defined(__PAM2__TempMesh__) */
