//
//  PAMViewController.m
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#import "PAMES2ViewController.h"
#include "Mat4x4f.h"
#include "PAMManifold.h"
#include "../HMesh/obj_load.h"
#include "RARotationManager.h"
#include "RAZoomManager.h"
#include "RATranslationManager.h"
#include "RABoundingBox.h"
#include "RAUnitSphere.h"
#include "RAES2OffScreenBuffer.h"
#include "glu.h"
#include "PAMSettingsManager.h"
#include "RAPolyLine.h"

using namespace std;
using namespace CGLA;
using namespace PAMMesh;
using namespace RAEngine;

@interface PAMES2ViewController()
{
    RAES2OffScreenBuffer* offScreenBuffer; //offscreen color and depth buffer
    
    Mat4x4f viewMatrix;
    Mat4x4f projectionMatrix;
    Vec3f viewVolumeCenter;
    
    RARotationManager* rotManager;
    RAZoomManager* zoomManager;
    RATranslationManager* translationManager;
}
@end

//Gestures
@interface PAMES2ViewController()
{
    UIPanGestureRecognizer* twoFingerTranslation;
}
@end

//Meshes
@interface PAMES2ViewController()
{
    PAMManifold* pamManifold;
    Bounds bounds;
    
    RABoundingBox* boundingBox;
    RAPolyLine* polyline1;
    vector<Vec3f> polyline1Data;
    RAPolyLine* polyline2;
    vector<Vec3f> polyline2Data;
}
@end


@implementation PAMES2ViewController

#pragma mark - View cycle and OpenGL setup
- (void)viewDidLoad
{
    [super viewDidLoad];
    [self setupGL];
    [self loadEmptyMeshData];
    [self addGestureRecognizersToView:self.view];
    
    // Custom initialization
    rotManager = new RARotationManager();
    zoomManager = new RAZoomManager();
    translationManager = new RATranslationManager();
    
    polyline1 = new RAPolyLine();
    polyline2 = new RAPolyLine();
    std::string vShader_Cplus([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"vsh"].UTF8String);
    std::string fShader_Cplus([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"fsh"].UTF8String);
    polyline1->setupShaders(vShader_Cplus, fShader_Cplus);
    polyline2->setupShaders(vShader_Cplus, fShader_Cplus);
}

- (void)viewDidUnload
{
    [super viewDidUnload];
    
    delete offScreenBuffer;
    delete pamManifold;
    delete boundingBox;
    delete rotManager;
    delete zoomManager;
    delete translationManager;
    delete polyline1;
    delete polyline2;
    
    ((GLKView *)self.view).context = nil;
    [EAGLContext setCurrentContext:nil];
}

-(void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    offScreenBuffer = new RAES2OffScreenBuffer();
    offScreenBuffer->createOffScreenBuffer(_glWidth, _glHeight);
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void)setupGL
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

#pragma mark - Gesture Recongnizers
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
    twoFingerTranslation = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handleTwoFingerPanGesture:)];
    twoFingerTranslation.minimumNumberOfTouches = 2;
    twoFingerTranslation.maximumNumberOfTouches = 2;
    [view addGestureRecognizer:twoFingerTranslation];
    
    //Tap
    UITapGestureRecognizer* tapGesture = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleTapGestureRecognizer:)];
    tapGesture.numberOfTapsRequired = 1;
    tapGesture.numberOfTouchesRequired = 1;
    [view addGestureRecognizer:tapGesture];
}

-(GestureState)gestureStateForRecognizer:(UIGestureRecognizer*)sender
{
    GestureState state;
    switch (sender.state) {
        case UIGestureRecognizerStateBegan:
            state = GestureState::Began;
            break;
        case UIGestureRecognizerStateChanged:
            state = GestureState::Changed;
            break;
        case UIGestureRecognizerStateEnded:
            state = GestureState::Ended;
            break;
        default:
            break;
    }
    return state;
}

-(void)handleTapGestureRecognizer:(UITapGestureRecognizer*)sender
{
//    Vec3f toucP;
//    [self modelCoordinates:toucP forGesture:sender];
}

-(void)handleOneFingerPanGesture:(UIPanGestureRecognizer*)sender
{
    CGPoint cocoaTouch = [sender locationInView:sender.view];
    Vec2i glTouch = Vec2i(cocoaTouch.x, [sender.view bounds].size.height - cocoaTouch.y);
    GestureState state = [self gestureStateForRecognizer:sender];
    rotManager->handlePanGesture(state, glTouch, viewVolumeCenter);
}

-(void)handleRotationGesture:(UIRotationGestureRecognizer*)sender
{
    GestureState state = [self gestureStateForRecognizer:sender];
    float rotaion = [sender rotation];
    rotManager->handleRotationGesture(state, rotaion, viewVolumeCenter);
}

-(void)handlePinchGesture:(UIPinchGestureRecognizer*)sender
{
    GestureState state = [self gestureStateForRecognizer:sender];
    zoomManager->handlePinchGesture(state, sender.scale);
}

-(void)handleTwoFingerPanGesture:(UIPanGestureRecognizer*)sender
{
    if (PAMSettingsManager::getInstance().transform)
    {
        CGPoint touchPoint = [self scaleTouchPoint:[sender locationInView:sender.view]
                                            inView:(GLKView*)sender.view];
        Vec3f rayStartWindow = Vec3f(touchPoint.x, touchPoint.y, 0);
        Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
        Vec3f rayOrigin;
        gluUnProjectf(rayStartWindow, projectionMatrix, viewport, rayOrigin);
        Vec3f axis(rayOrigin[0], rayOrigin[1], 0);
        
        GestureState state = [self gestureStateForRecognizer:sender];
        translationManager->handlePanGesture(state, axis);
    }
    else
    {
        if (sender.state == UIGestureRecognizerStateBegan)
        {
            polyline1Data.clear();
            polyline2Data.clear();
            polyline1->enabled = true;
            polyline2->enabled = true;
        }
        
        if (sender.numberOfTouches == 2)
        {
            CGPoint touchPoint1 = [self scaleTouchPoint:[sender locationOfTouch:0 inView:(GLKView*)sender.view]
                                                 inView:(GLKView*)sender.view];
            CGPoint touchPoint2 = [self scaleTouchPoint:[sender locationOfTouch:1 inView:(GLKView*)sender.view]
                                                 inView:(GLKView*)sender.view];
            
            Vec3f rayOrigin1, rayOrigin2;
            BOOL result1 = [self rayOrigin:rayOrigin1 forTouchPoint:touchPoint1];
            BOOL result2 = [self rayOrigin:rayOrigin2 forTouchPoint:touchPoint2];
            if (!result1 || !result2) {
                RA_LOG_WARN("Couldn't determine touch area");
                return;
            }
            
            polyline1Data.push_back(rayOrigin1);
            polyline1->bufferVertexDataToGPU(polyline1Data, Vec4uc(255,0,0,255));
            polyline2Data.push_back(rayOrigin2);
            polyline2->bufferVertexDataToGPU(polyline2Data, Vec4uc(255,0,0,255));
        }

        if (sender.state == UIGestureRecognizerStateEnded ||
            sender.state == UIGestureRecognizerStateFailed ||
            sender.state == UIGestureRecognizerStateCancelled)
        {
            polyline1->enabled = false;
            polyline2->enabled = false;

            if (pamManifold != nullptr) {
                delete pamManifold;
            }
            pamManifold = new PAMManifold();
            pamManifold->setupShaders();
            pamManifold->createBody(polyline1Data, polyline2Data);
            pamManifold->bufferVertexDataToGPU();
            pamManifold->enabled = true;
            
            GLfloat zNear = 1.0;
            GLfloat newOriginZ = -1*(zNear + bounds.radius);
            GLfloat curOriginZ = bounds.center[2];
            
            pamManifold->translate(Vec3f(0, 0, newOriginZ - curOriginZ));
            
            [self setupBoundingBox];
        }
    }
}

//Load empty workspace
-(void)loadEmptyMeshData
{
    [self setPaused:YES];
    bounds = {Vec3f(-1,-1,-1), Vec3f(1,1,1), Vec3f(0,0,0), 2*sqrtf(3.0f)/2.0f};

    GLfloat zNear = 1.0;
    GLfloat newOriginZ = -1*(zNear + bounds.radius);
    GLfloat curOriginZ = bounds.center[2];

    Vec3f tV = Vec3f(0, 0, newOriginZ - curOriginZ);
    viewVolumeCenter = tV + bounds.center;
    
    [self setupBoundingBox];

    [self setPaused:NO];
}

//Load initial mesh from OBJ file
-(void)loadMeshData
{
    [self setPaused:YES]; //pause rendering
    
    //Load obj file
    NSString* objPath = [[NSBundle mainBundle] pathForResource:@"shuttle" ofType:@"obj"];
    if (objPath) {
        if (pamManifold != nullptr) {
            delete pamManifold;
        }
        pamManifold = new PAMManifold();
        pamManifold->loadObjFile(objPath.UTF8String);
        pamManifold->setupShaders();
        pamManifold->bufferVertexDataToGPU();

        bounds = pamManifold->getBoundingBox();
        
        GLfloat zNear = 1.0f;
        GLfloat newOriginZ = -1 * (zNear + bounds.radius);
        GLfloat curOriginZ = bounds.center[2];
        
        Vec3f tV = Vec3f(0, 0, newOriginZ - curOriginZ);
        pamManifold->translate(tV);
        viewVolumeCenter = tV + bounds.center;
        [self setupBoundingBox];
    }
    
    [self setPaused:NO];
}

-(void)setupBoundingBox
{
    if (boundingBox != nullptr) {
        delete boundingBox;
    }
    
    boundingBox = new RABoundingBox(bounds.minBound, bounds.maxBound);
    std::string vShader_Cplus([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"vsh"].UTF8String);
    std::string fShader_Cplus([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"fsh"].UTF8String);
    boundingBox->setupShaders(vShader_Cplus, fShader_Cplus);
    boundingBox->bufferVertexDataToGPU();
    boundingBox->translate(viewVolumeCenter - bounds.center);
    boundingBox->enabled = true;
}

#pragma mark - OpenGL Drawing

-(void)update
{
    float diam = bounds.radius * 2.0f;
    float rad = bounds.radius;
    GLfloat aspect = (GLfloat)_glWidth / (GLfloat)_glHeight;
    float scale = 1.0f/zoomManager->getScaleFactor();
    
    GLfloat left = viewVolumeCenter[0] - rad*aspect*scale;
    GLfloat right = viewVolumeCenter[0] + rad*aspect*scale;
    GLfloat bottom = viewVolumeCenter[1] - rad*scale;
    GLfloat top = viewVolumeCenter[1] + rad*scale;

    projectionMatrix = ortho_Mat4x4f(left, right, bottom, top, 1.0f, 1.0f + diam);
    viewMatrix = translationManager->getTranslationMatrix() * rotManager->getRotationMatrix();
}

-(void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    
    if (pamManifold != nullptr) {
        pamManifold->projectionMatrix = projectionMatrix;
        pamManifold->viewMatrix = viewMatrix;
        pamManifold->draw();
    }
    
    if (boundingBox != nullptr) {
        boundingBox->projectionMatrix = projectionMatrix;
        boundingBox->viewMatrix = viewMatrix;
        boundingBox->draw();
    }
    
    polyline1->projectionMatrix = projectionMatrix;
    polyline1->viewMatrix = viewMatrix;
    polyline1->draw();
    
    polyline2->projectionMatrix = projectionMatrix;
    polyline2->viewMatrix = viewMatrix;
    polyline2->draw();
}

#pragma mark - Offscreen buffer
-(GLubyte*)renderToOffscreenDepthBuffer:(PAMManifold*) mesh {
    //Preserve previous GL state
    GLboolean wasBlendEnabled;
    glGetBooleanv(GL_BLEND, &wasBlendEnabled);
    
    GLboolean wasDepthEnabled;
    glGetBooleanv(GL_DEPTH_TEST, &wasDepthEnabled);
    
    GLfloat clearColor[4];
    glGetFloatv(GL_COLOR_CLEAR_VALUE, clearColor);
    
    offScreenBuffer->bind();
    glViewport(0, 0, _glWidth, _glHeight);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    
    //    for (RAMesh* mesh : meshesArray) {
    mesh->viewMatrix = viewMatrix;
    mesh->projectionMatrix = projectionMatrix;
    mesh->drawToDepthBuffer();
    //    }
    
    GLubyte* pixelData = new GLubyte[4*_glWidth*_glHeight];
    glReadPixels(0, 0, _glWidth, _glHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixelData);
    
    //Restore previous GL state
    if (wasBlendEnabled) {
        glEnable(GL_BLEND);
    } else {
        glDisable(GL_BLEND);
    }
    
    if (wasDepthEnabled) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
    
    offScreenBuffer->unbind();
    
    return pixelData;
}

//Return depth value written int colorbuffer using shaders
//-1 is returned if depth is unkown (clicked on background pixels)
-(float)depthForPoint:(CGPoint)touchPoint depthBuffer:(GLubyte*)pixelData
{
    int index = touchPoint.y * _glWidth + touchPoint.x;
    int index4 = index * 4;
    
    GLubyte r = pixelData[index4];
    GLubyte g = pixelData[index4 + 1];
    GLubyte b = pixelData[index4 + 2];
    GLubyte a = pixelData[index4 + 3];
    
    if (r != 0 && g != 0 && b != 0) {
        Vec4f color = Vec4f(r, g, b, a);
        float depth = dot(color, Vec4f(1.0, 1/255.0, 1/65025.0, 1/16581375.0));
        depth = depth/255.0;
        return depth;
    }
    
    return -1;
}

-(BOOL)modelCoordinates:(Vec3f&)modelCoord forGesture:(UIGestureRecognizer*)sender
{
    CGPoint touchPoint = [self touchPointFromGesture:sender];
    GLubyte* pixelData = [self renderToOffscreenDepthBuffer:pamManifold];
    BOOL result  = [self modelCoordinates:modelCoord forTouchPoint:touchPoint depthBuffer:pixelData];
    delete pixelData;
    return result;
}

//Convert window coordinates into world coordinates.
//Touchpoint is in the form of (touchx, touchy). Depth is extracted from given depth buffer information
-(BOOL)modelCoordinates:(Vec3f&)objectCoord3 forTouchPoint:(CGPoint)touchPoint depthBuffer:(GLubyte*)pixelData
{
    float depth = [self depthForPoint:touchPoint depthBuffer:pixelData];
    
    if (depth >= 0) {
        Vec3f windowCoord3 = Vec3f(touchPoint.x, touchPoint.y, depth);
        Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
        int result = gluUnProjectf(windowCoord3, projectionMatrix*viewMatrix, viewport, objectCoord3);
        if (result != 0) {
            return YES;
        }
    }
    return NO;
}

//Get scaled and flipped touch coordinates from touch gesture
-(CGPoint)touchPointFromGesture:(UIGestureRecognizer*)sender
{
    return [self scaleTouchPoint:[sender locationInView:sender.view] inView:(GLKView*)sender.view];
}

//Get scaled and flipped touch coordinates from touch point coordinates in a view
-(CGPoint)scaleTouchPoint:(CGPoint)touchPoint inView:(GLKView*)view
{
    CGFloat scale = view.contentScaleFactor;
    
    touchPoint.x = floorf(touchPoint.x * scale);
    touchPoint.y = floorf(touchPoint.y * scale);
    touchPoint.y = floorf(view.drawableHeight - touchPoint.y);
    
    return touchPoint;
}

-(BOOL)rayOrigin:(Vec3f&)rayOrigin forTouchPoint:(CGPoint)touchPoint
{
    Vec3f rayDir;
    return [self rayOrigin:rayOrigin rayDirection:rayDir forTouchPoint:touchPoint];
}

-(BOOL)rayOrigin:(Vec3f&)rayOrigin rayDirection:(Vec3f&)rayDirection forTouchPoint:(CGPoint)touchPoint
{
    Vec3f rayStartWindow = Vec3f(touchPoint.x, touchPoint.y, 0.5);
    Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
    int result = gluUnProjectf(rayStartWindow, projectionMatrix*viewMatrix, viewport, rayOrigin);
    if (result == 0 ) {
        return NO;
    }
    rayDirection = Vec3f(invert_affine(rotManager->getRotationMatrix()) * Vec4f(0,0,-1,0));
    return YES;
}

-(BOOL)rayOrigin:(Vec3f&)rayOrigin rayDirection:(Vec3f&)rayDirection forGesture:(UIGestureRecognizer*)gesture
{
    CGPoint touchPoint = [self scaleTouchPoint:[gesture locationInView:gesture.view] inView:(GLKView*)gesture.view];
    return [self rayOrigin:rayOrigin rayDirection:rayDirection forTouchPoint:touchPoint];
}


@end
