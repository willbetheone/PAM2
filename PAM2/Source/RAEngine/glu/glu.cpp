//
//  glu.cpp
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-07.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#include "glu.h"

int gluUnProjectf(CGLA::Vec3f win, CGLA::Mat4x4f modelviewProjection, CGLA::Vec4f viewport, CGLA::Vec3f& objectCoordinate)
{
    //Transformation matrices
    CGLA::Mat4x4f modelviewProjectionInverse;
    CGLA::Vec4f inVec;
    CGLA::Vec4f outVec;
    
    //Now compute the inverse of matrix A
    
    modelviewProjectionInverse = invert_affine(modelviewProjection);
    
    //Transformation of normalized coordinates between -1 and 1
    inVec[0]=(win[0]-(float)viewport[0])/(float)viewport[2]*2.0-1.0;
    inVec[1]=(win[1]-(float)viewport[1])/(float)viewport[3]*2.0-1.0;
    inVec[2]=2.0*win[2]-1.0;
    inVec[3]=1.0;
    //Objects coordinates
    outVec = modelviewProjectionInverse * inVec;
    
    if(outVec[3]==0.0)
        return 0;
    
    outVec[3]=1.0/outVec[3];
    objectCoordinate[0]=outVec[0]*outVec[3];
    objectCoordinate[1]=outVec[1]*outVec[3];
    objectCoordinate[2]=outVec[2]*outVec[3];
    return 1;
}