//
//  PAMViewController.m
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#import "PAMViewController.h"
#include "Mat4x4f.h"
#include "PAMManifold.h"
#include "../HMesh/obj_load.h"
#include "TempMesh.h"

using namespace CGLA;
using namespace PAMMesh;
using namespace TempToDelete;

@interface PAMViewController () {
    //Off Screen framebuffers and renderbuffers
//    GLuint _offScreenFrameBuffer;
//    GLuint _offScreenColorBuffer;
//    GLuint _offScreenDepthBuffer;
    
    Mat4x4f viewMatrix;
    Mat4x4f projectionMatrix;
    
//    PAMManifold* pamManifold;
    TempMesh* tempMesh;
}

@end

@implementation PAMViewController

- (id)init
{
    self = [super init];
    if (self) {
        // Custom initialization
    }
    return self;
}
#pragma mark - View cycle and OpenGL setup
- (void)viewDidLoad
{
    [super viewDidLoad];
    [self setupGL];

    tempMesh = new TempMesh();
    NSString* vShader = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"vsh"];
    NSString* fShader = [[NSBundle mainBundle] pathForResource:@"Shader" ofType:@"fsh"];
    std::string vShader_Cplus([vShader UTF8String]);
    std::string fShader_Cplus([fShader UTF8String]);
    tempMesh->setShaders(vShader_Cplus, fShader_Cplus);
    tempMesh->setVertexData();
}

- (void)viewDidUnload
{
    [super viewDidUnload];
    
    delete tempMesh;
    
    ((GLKView *)self.view).context = nil;
    [EAGLContext setCurrentContext:nil];
    
//    glDeleteRenderbuffers(1, &_offScreenColorBuffer);
//    glDeleteRenderbuffers(1, &_offScreenDepthBuffer);
//    glDeleteFramebuffers(1, &_offScreenFrameBuffer);
}

//-(void)viewDidAppear:(BOOL)animated
//{
//    [super viewDidAppear:animated];
//    [self createOffScreenBuffer];
//}

//-(void) createOffScreenBuffer
//{
//    //Create additional Buffers
//    //Create framebuffer and attach color/depth renderbuffers
//    glGenFramebuffers(1, &_offScreenFrameBuffer);
//    glBindFramebuffer(GL_FRAMEBUFFER, _offScreenFrameBuffer);
//    
//    glGenRenderbuffers(1, &_offScreenColorBuffer);
//    glBindRenderbuffer(GL_RENDERBUFFER, _offScreenColorBuffer);
//    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, _glWidth, _glHeight);
//    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _offScreenColorBuffer);
//    
//    glGenRenderbuffers(1, &_offScreenDepthBuffer);
//    glBindRenderbuffer(GL_RENDERBUFFER, _offScreenDepthBuffer);
//    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, _glWidth, _glHeight);
//    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _offScreenDepthBuffer);
//    
//    int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
//    if (status != GL_FRAMEBUFFER_COMPLETE) {
//        NSLog(@"[ERROR] Couldnt create offscreen buffer");
//    }
//    
//    glBindFramebuffer(GL_FRAMEBUFFER, 0);
//}

-(void)setupGL
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}


- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

////Load initial mesh from OBJ file
//-(void)loadMeshData {
//    
//    [self setPaused:YES]; //pause rendering
//    if (pamManifold != nullptr) {
//        delete pamManifold;
//    }
//    pamManifold = new PAMManifold();
//    
//    //Load obj file
//    NSString* objPath = [[NSBundle mainBundle] pathForResource:@"HAND PAM 15" ofType:@"obj"];
//    HMesh::obj_load(objPath.UTF8String, *pamManifold);
//    
//    [self setPaused:NO];
//}


#pragma mark - OpenGL Drawing

-(void)update
{
    projectionMatrix = ortho_Mat4x4f(-1, 1, -1*_aspectRatio, 1*_aspectRatio, -4, 4);
    viewMatrix = identity_Mat4x4f();
}

-(void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    
    tempMesh->projectionMatrix = projectionMatrix;
    tempMesh->viewMatrix = viewMatrix;
    tempMesh->draw();
}


@end
