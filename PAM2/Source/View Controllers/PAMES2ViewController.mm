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

#define SHOW_DEBUG_LINES 0

typedef enum {
    TOUCHED_NONE,
    TOUCHED_MODEL,
    TOUCHED_BACKGROUND
} DrawingState;

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
    
    DrawingState _drawingState;
    Vec3f firstPoint;
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
    string* vShader;
    string* fShader;
    
    vector<RAPolyLine*> debugPolylines;
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
    vShader = new string([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"vsh"].UTF8String);
    fShader = new string([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"fsh"].UTF8String);
    polyline1->setupShaders(*vShader, *fShader);
    polyline2->setupShaders(*vShader, *fShader);
    
    PAMSettingsManager::getInstance().transform = false;
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
    //Two finger tap
    UITapGestureRecognizer* twoFingerTap = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleTwoFingeTapGesture:)];
    twoFingerTap.numberOfTapsRequired = 1;
    twoFingerTap.numberOfTouchesRequired = 2;
    [view addGestureRecognizer:twoFingerTap];
    
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

-(void)handleTwoFingeTapGesture:(UIGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    PAMSettingsManager::getInstance().transform  = !PAMSettingsManager::getInstance().transform;
}

-(void)handleOneFingerPanGesture:(UIPanGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    if (PAMSettingsManager::getInstance().transform)
    {
        CGPoint cocoaTouch = [sender locationInView:sender.view];
        Vec2i glTouch = Vec2i(cocoaTouch.x, [sender.view bounds].size.height - cocoaTouch.y);
        GestureState state = [self gestureStateForRecognizer:sender];
        rotManager->handlePanGesture(state, glTouch, viewVolumeCenter);
    }
    else
    {
        if (sender.state == UIGestureRecognizerStateBegan)
        {
            _drawingState = TOUCHED_NONE;
            
            CGPoint touchPoint = [self touchPointFromGesture:sender];
            Vec3f rayOrigin;
            BOOL result = [self rayOrigin:rayOrigin forTouchPoint:touchPoint];
            if (!result) {
                RA_LOG_WARN("Touched background");
                return;
            }
            polyline1Data.clear();
            polyline1Data.push_back(rayOrigin);
            
            //Start branch creation
            GLubyte* pixelData = [self renderToOffscreenDepthBuffer:pamManifold];
            float depth = [self depthForPoint:touchPoint depthBuffer:pixelData];
            
            Vec3f modelCoord;
            if (depth < 0)
            { //clicked on background
                BOOL result = [self modelCoordinates:modelCoord
                                       forTouchPoint:Vec3f(touchPoint.x, touchPoint.y, 0)];
                if (!result) {
                    RA_LOG_WARN("Touched background");
                    return;
                }
                _drawingState = TOUCHED_BACKGROUND;
            }
            else
            { //clicked on a model
                BOOL result = [self modelCoordinates:modelCoord
                                       forTouchPoint:Vec3f(touchPoint.x, touchPoint.y, depth)];
                if (!result) {
                    RA_LOG_WARN("Touched background");
                    return;
                }
                _drawingState = TOUCHED_MODEL;
            }
            firstPoint = modelCoord;
        }
        else if (sender.state == UIGestureRecognizerStateChanged)
        {
            //Add touch point to a line
            CGPoint touchPoint = [self touchPointFromGesture:sender];
            Vec3f rayOrigin;
            BOOL result = [self rayOrigin:rayOrigin forTouchPoint:touchPoint];
            if (!result) {
                RA_LOG_WARN("Touched background");
                return;
            }
            polyline1Data.push_back(rayOrigin);
            
            if (polyline1Data.size() > 1) {
                polyline1->bufferVertexDataToGPU(polyline1Data, Vec4uc(255,0,0,255), GL_LINE_STRIP);
                polyline1->enabled = true;
            }
        }
        else if (sender.state == UIGestureRecognizerStateEnded)
        {
            polyline1->enabled = false;
            Vec3f modelCoord;
            
            BOOL touchedModelStart = _drawingState == TOUCHED_MODEL;

            CGPoint touchPoint = [self touchPointFromGesture:sender];
            Vec3f rayOrigin;
            BOOL result = [self rayOrigin:rayOrigin forTouchPoint:touchPoint];
            if (!result) {
                RA_LOG_WARN("Touched background");
                return;
            }
            polyline1Data.push_back(rayOrigin);
            
            float touchSize = [self touchSizeForGesture:sender];
            pamManifold->endCreateBranchBended(polyline1Data,
                                               firstPoint,
                                               touchedModelStart,                                               
                                               false,
                                               touchSize);
            _drawingState = TOUCHED_NONE;
        }
    }
}

-(void)handleRotationGesture:(UIRotationGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    GestureState state = [self gestureStateForRecognizer:sender];
    float rotaion = [sender rotation];
    rotManager->handleRotationGesture(state, rotaion, viewVolumeCenter);
}

-(void)handlePinchGesture:(UIPinchGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    if (PAMSettingsManager::getInstance().transform)
    {
        GestureState state = [self gestureStateForRecognizer:sender];
        zoomManager->handlePinchGesture(state, sender.scale);
    } else {
        if (sender.state == UIGestureRecognizerStateBegan)
        {
            CGPoint touchPoint1 = [self scaleTouchPoint:[sender locationOfTouch:0 inView:(GLKView*)sender.view]
                                                 inView:(GLKView*)sender.view];
            CGPoint touchPoint2 = [self scaleTouchPoint:[sender locationOfTouch:1 inView:(GLKView*)sender.view]
                                                 inView:(GLKView*)sender.view];
            
            GLubyte* pixelData = [self renderToOffscreenDepthBuffer:pamManifold];
            float depth1 = [self depthForPoint:touchPoint1 depthBuffer:pixelData];
            float depth2 = [self depthForPoint:touchPoint2 depthBuffer:pixelData];
            
            UIGestureRecognizer* g = (UIGestureRecognizer*)sender;
            float touchSize = 2 * [self touchSizeForGesture:g];
            
            if (depth1 < 0 || depth2 < 0)
            {
                //on of the fingers touched background. Start scaling.
                if (depth1 < 0 && depth2 < 0)
                {
                    Vec3f rayOrigin;
                    if (![self rayOrigin:rayOrigin forTouchPoint:touchPoint1]) {
                        RA_LOG_WARN("Couldn't determine touch area");
                        return;
                    }
                    bool anisotropic = PAMSettingsManager::getInstance().sculptScalingType == PAMSettingsManager::ScultpScalingType::Silhouette;
                    pamManifold->startScalingSingleRib(rayOrigin,false, sender.scale, sender.velocity, touchSize, anisotropic);
                }
                else if (depth1 < 0)
                {
                    Vec3f rayOrigin;
                    if (![self rayOrigin:rayOrigin forTouchPoint:touchPoint1]) {
                        RA_LOG_WARN("Couldn't determine touch area");
                        return;
                    }
                    bool anisotropic = PAMSettingsManager::getInstance().sculptScalingType == PAMSettingsManager::ScultpScalingType::Silhouette;
                    pamManifold->startScalingSingleRib(rayOrigin, true, sender.scale, sender.velocity, touchSize,anisotropic);
                }
                else
                {
                    Vec3f rayOrigin;
                    if (![self rayOrigin:rayOrigin forTouchPoint:touchPoint2]) {
                        RA_LOG_WARN("Couldn't determine touch area");
                        return;
                    }
                    bool anisotropic = PAMSettingsManager::getInstance().sculptScalingType == PAMSettingsManager::ScultpScalingType::Silhouette;
                    pamManifold->startScalingSingleRib(rayOrigin,true,sender.scale,sender.velocity,touchSize, anisotropic);
                }
            }
            else if (depth1 >= 0 && depth2 >= 0)
            {
                //touched the model so start bump creation
                Vec3f modelCoord1, modelCoord2;
                Vec3f worldTouchPoint1 = Vec3f(touchPoint1.x, touchPoint1.y, depth1);
                Vec3f worldTouchPoint2 = Vec3f(touchPoint2.x, touchPoint2.y, depth2);
                [self modelCoordinates:modelCoord1 forTouchPoint:worldTouchPoint1];
                [self modelCoordinates:modelCoord2 forTouchPoint:worldTouchPoint2];
                float distanceBetweenFingers = (modelCoord2 - modelCoord1).length();
                
                CGPoint touchPoint = CGPointMake(floorf(0.5*(touchPoint1.x +touchPoint2.x)),
                                                 floorf(0.5*(touchPoint1.y +touchPoint2.y)));
                float depth = [self depthForPoint:touchPoint depthBuffer:pixelData];
                Vec3f modelCoord;
                [self modelCoordinates:modelCoord forTouchPoint:Vec3f(touchPoint.x, touchPoint.y, depth)];
                float brushSize = distanceBetweenFingers/2;
    //            [_pMesh startBumpCreationAtPoint:modelCoord
    //                                   brushSize:brushSize
    //                                  brushDepth:pinch.scale];
            }
        } else if (sender.state == UIGestureRecognizerStateChanged) {
            if (pamManifold->modState == PAMManifold::Modification::SCULPTING_BUMP_CREATION) {
    //            [_pMesh continueBumpCreationWithBrushDepth:pinch.scale];
            } else {
                pamManifold->changeScalingSingleRib(sender.scale);
            }
        } else if (sender.state == UIGestureRecognizerStateEnded) {
            if (pamManifold->modState == PAMManifold::Modification::SCULPTING_BUMP_CREATION) {
    //            [_pMesh endBumpCreation];
            } else {
                pamManifold->endScalingSingleRib(sender.scale);
            }
        }
    }
}

-(void)handleTwoFingerPanGesture:(UIPanGestureRecognizer*)sender
{
    if (PAMSettingsManager::getInstance().transform)
    {
        if (![self modelIsLoaded]) {
            return;
        }

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
            polyline2Data.push_back(rayOrigin2);
            
            if (polyline1Data.size() > 1 && polyline2Data.size() > 1) {
                polyline1->bufferVertexDataToGPU(polyline1Data, Vec4uc(255,0,0,255), GL_LINE_STRIP);
                polyline2->bufferVertexDataToGPU(polyline2Data, Vec4uc(255,0,0,255), GL_LINE_STRIP);
                polyline1->enabled = true;
                polyline2->enabled = true;
            }
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
            vector<vector<Vec3f>> debugRibs;
            if (pamManifold->createBody(polyline1Data, polyline2Data, 0, debugRibs, false))
            {
                pamManifold->enabled = true;
                
                GLfloat zNear = 1.0;
                GLfloat newOriginZ = -1*(zNear + bounds.radius);
                GLfloat curOriginZ = bounds.center[2];
                
                pamManifold->translate(Vec3f(0, 0, newOriginZ - curOriginZ));
                
                [self setupBoundingBox];
                
#if SHOW_DEBUG_LINES
                clearVector(debugPolylines);
                for (int i = 0; i < debugRibs.size();i++) {
                    vector<Vec3f> rib = debugRibs[i];
                    if (rib.size() < 2) {
                        continue;
                    }
                    RAPolyLine* p = new RAPolyLine();
                    p->setupShaders(*vShader, *fShader);
                    p->bufferVertexDataToGPU(rib, Vec4uc(0,0,255,255), Vec4uc(0,255,0,255), GL_LINES);
                    
                    p->translate(Vec3f(0, 0, newOriginZ - curOriginZ));
                    p->enabled = true;
                    debugPolylines.push_back(p);
                }
//                PAMSettingsManager::getInstance().transform = true;
#endif

            }
            else
            {
                delete pamManifold;
                pamManifold = nullptr;
            }
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
        pamManifold->setupShaders();
        pamManifold->loadObjFile(objPath.UTF8String);

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
        pamManifold->updateMesh();
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
    
#if SHOW_DEBUG_LINES
    for (RAPolyLine* p : debugPolylines) {
        p->projectionMatrix = projectionMatrix;
        p->viewMatrix = viewMatrix;
        p->draw();
    }
#endif
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

-(BOOL)modelCoordinates:(Vec3f&)objectCoord3 forTouchPoint:(Vec3f)windowCoord3 {
    Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
    int result = gluUnProjectf(windowCoord3, projectionMatrix*viewMatrix, viewport, objectCoord3);
    if (result != 0) {
        return YES;
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
    Vec3f rayStartWindow = Vec3f(touchPoint.x, touchPoint.y, 0);
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

-(float)touchSizeForGesture:(UIGestureRecognizer*)gesture
{
    float touchSizeMM;
    if ([gesture respondsToSelector:@selector(touchSize)])  {
        touchSizeMM = [[gesture valueForKey:@"touchSize"] floatValue];
    } else {
        touchSizeMM = 10;
    }
    float touchSize = [self touchSizeForFingerSize:touchSizeMM];
    return touchSize;
}

-(float)touchSizeForFingerSize:(float)touchSizeMM {
    const float mmToPx = 2048.0f/240.0f; //2048 px for 240 mm for retina display
    float touchSizePx = touchSizeMM * mmToPx;
    
    Vec3f modelCoord, modelCoord2, rayDir, rayDir2;
    [self rayOrigin:modelCoord forTouchPoint:CGPointMake(touchSizePx, 0)];
    [self rayOrigin:modelCoord2 forTouchPoint:CGPointMake(0, 0)];
    
    float touchSize = length(modelCoord - modelCoord2);
    return touchSize;
}

-(BOOL)modelIsLoaded
{
    return pamManifold != nullptr && pamManifold->enabled;
}

template <class C>
void clearVector(std::vector<C*>& inputvector)
{
    for (int i = 0; i < inputvector.size(); i++) {
        delete inputvector[i];
    }
    inputvector.clear();
}


@end
