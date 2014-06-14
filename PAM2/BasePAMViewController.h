//
//  BasePAMViewController.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>

@interface BasePAMViewController : GLKViewController {
    GLsizei _glWidth;
    GLsizei _glHeight;
    GLfloat _aspectRatio;
}

@end
