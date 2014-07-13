//
//  glu.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-07.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__glu__
#define __PAM2__glu__

#include <iostream>
#include "Vec3f.h"
#include "Vec4f.h"
#include "Mat4x4f.h"

int gluUnProjectf(CGLA::Vec3f win, CGLA::Mat4x4f modelviewProjection, CGLA::Vec4f viewport, CGLA::Vec3f& objectCoordinate);

#endif /* defined(__PAM2__glu__) */
