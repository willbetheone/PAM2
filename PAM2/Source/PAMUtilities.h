//
//  PAMUtilities.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-23.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#ifndef __PAM2__PAMUtilities__
#define __PAM2__PAMUtilities__

#include <iostream>
#include "Vec2f.h"
#include <vector>

namespace PAMMesh
{
    bool getRibWidths(const std::vector<CGLA::Vec2f>& line1,
                      const std::vector<CGLA::Vec2f>& line2,
                      const std::vector<CGLA::Vec2f>& center,
                      const std::vector<CGLA::Vec2f>& norms,
                      std::vector<float>& ribWidth);
}

#endif /* defined(__PAM2__PAMUtilities__) */
