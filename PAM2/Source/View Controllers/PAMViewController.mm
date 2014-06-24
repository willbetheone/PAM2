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
#include "RARotationManager.h"
#include "RAZoomManager.h"
#include "RATranslationManager.h"

using namespace CGLA;
using namespace PAMMesh;
using namespace RAEngine;

@interface PAMViewController () {
    //Off Screen framebuffers and renderbuffers
//    GLuint _offScreenFrameBuffer;
//    GLuint _offScreenColorBuffer;
//    GLuint _offScreenDepthBuffer;
    
    Mat4x4f viewMatrix;
    Mat4x4f projectionMatrix;
    
    PAMManifold* pamManifold;
    
    RARotationManager* rotManager;
    RAZoomManager* zoomManager;
    RATranslationManager* translationManager;
    
    RABoundingBox boundingBox;
}

@end

@implementation PAMViewController

- (id)init
{
    self = [super init];
    if (self) {
        // Custom initialization
        rotManager = new RARotationManager();
        zoomManager = new RAZoomManager();
        translationManager = new RATranslationManager();
        boundingBox = RABoundingBox();
    }
    return self;
}

#pragma mark - View cycle and OpenGL setup
- (void)viewDidLoad
{
    [super viewDidLoad];
    [self setupGL];
    [self loadMeshData];
    [self addGestureRecognizersToView:self.view];
}

- (void)viewDidUnload
{
    [super viewDidUnload];
    
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


-(void)addGestureRecognizersToView:(UIView*)view
{
    // Side Rotation
    UIPanGestureRecognizer* oneFingerPanning = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handleOneFingerPanGesture:)];
    oneFingerPanning.maximumNumberOfTouches = 1;
    [view addGestureRecognizer:oneFingerPanning];
    
    //Rotate along Z-axis
    UIRotationGestureRecognizer* rotationInPlaneOfScreen = [[UIRotationGestureRecognizer alloc] initWithTarget:self action:@selector(handleRotationGesture:)];
    [view addGestureRecognizer:rotationInPlaneOfScreen];

    
    //Zoom
    UIPinchGestureRecognizer* pinchToZoom = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(handlePinchGesture:)];
    [view addGestureRecognizer:pinchToZoom];
    
    //Translation
    UIPanGestureRecognizer* twoFingerTranslation = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handleTwoFingerPanGesture:)];
    twoFingerTranslation.minimumNumberOfTouches = 2;
    twoFingerTranslation.maximumNumberOfTouches = 2;
    [view addGestureRecognizer:twoFingerTranslation];
}

-(void)handleOneFingerPanGesture:(UIGestureRecognizer*)sender
{
    rotManager->handlePanGesture(sender);
}

-(void)handleRotationGesture:(UIGestureRecognizer*)sender
{
    rotManager->handleRotationGesture(sender);
}

-(void)handlePinchGesture:(UIGestureRecognizer*)sender
{
    zoomManager->handlePinchGesture(sender);
}

-(void)handleTwoFingerPanGesture:(UIGestureRecognizer*)sender
{
    translationManager->handlePanGesture(sender);
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

//Load initial mesh from OBJ file
-(void)loadMeshData {
    
    [self setPaused:YES]; //pause rendering
    if (pamManifold != nullptr) {
        delete pamManifold;
    }
    pamManifold = new PAMManifold();
    
    //Load obj file
    NSString* objPath = [[NSBundle mainBundle] pathForResource:@"HAND PAM 15" ofType:@"obj"];
    if (objPath) {
        pamManifold->loadObjFile(objPath.UTF8String);
        pamManifold->setupShaders();
        pamManifold->bufferVertexDataToGPU();
        boundingBox = pamManifold->getBoundingBox();
    }
    
    [self setPaused:NO];
}

#pragma mark - OpenGL Drawing

-(void)update
{
    
    float diam = boundingBox.radius*2;
    Vec3f c = boundingBox.center;
    GLfloat zNear = 1.0;
    GLfloat zFar = zNear + diam;

    GLfloat left = c[0] - diam;
    GLfloat right = c[0] + diam;
    GLfloat bottom = c[1] - diam;
    GLfloat top = c[1] + diam;

    GLfloat aspect = (GLfloat)_glWidth / (GLfloat)_glHeight;

    if ( aspect < 1.0 ) { // window taller than wide
      bottom /= aspect;
      top /= aspect;
    } else {
      left *= aspect;
      right *= aspect;
    }
    
//    GLfloat aspectRatio = (GLfloat)_glHeight / (GLfloat)_glWidth;
    float scale = 1.0f/zoomManager->getScaleFactor();
//    float scale = 1.0f;
//    float bboxRadius = boundingBox.radius;
//    projectionMatrix = ortho_Mat4x4f(-1*scale*bboxRadius,
//                                      1*scale*bboxRadius,
//                                     -1*aspectRatio*scale*bboxRadius,
//                                      1*aspectRatio*scale*bboxRadius,
//                                     -1*2*bboxRadius,
//                                      1*2*bboxRadius);
    
    projectionMatrix = ortho_Mat4x4f(left*scale, right*scale, bottom*scale, top*scale, -1*zFar, zFar);
    

    viewMatrix = identity_Mat4x4f();
    viewMatrix *= translationManager->getTranslationMatrix();
    viewMatrix *= rotManager->getRotationMatrix();
}

-(void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    
    pamManifold->projectionMatrix = projectionMatrix;
    pamManifold->viewMatrix = viewMatrix;
    pamManifold->draw();
}

@end
