//
//  BasePAMViewController.m
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#import "BasePAMViewController.h"

@interface BasePAMViewController ()

@end

@implementation BasePAMViewController

- (id)init
{
    self = [super init];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    self.navigationController.navigationBarHidden = YES;
    self.preferredFramesPerSecond = 30;
    
    GLKView* view = [[GLKView alloc] initWithFrame:CGRectMake(0, 0, self.view.frame.size.width, self.view.frame.size.height)];
    view.multipleTouchEnabled = YES;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    view.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    [EAGLContext setCurrentContext:view.context];

    self.view = view;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)didRotateFromInterfaceOrientation:(UIInterfaceOrientation)fromInterfaceOrientation {
    _glWidth = ((GLKView*)self.view).drawableWidth;
    _glHeight = ((GLKView*)self.view).drawableHeight;
    _aspectRatio = (GLfloat)_glHeight / (GLfloat)_glWidth;
}

-(void)viewDidAppear:(BOOL)animated {
    [super viewDidAppear:animated];
    _glWidth = ((GLKView*)self.view).drawableWidth;
    _glHeight = ((GLKView*)self.view).drawableHeight;
    _aspectRatio = (GLfloat)_glHeight / (GLfloat)_glWidth;
}

@end
