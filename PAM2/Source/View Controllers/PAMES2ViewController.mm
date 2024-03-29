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
#include "../HMesh/obj_save.h"
#include "RARotationManager.h"
#include "RAZoomManager.h"
#include "RATranslationManager.h"
#include "RABoundingBox.h"
#include "RAUnitSphere.h"
#include "RAES2OffScreenBuffer.h"
#include "glu.h"
#include "PAMSettingsManager.h"
#include "RAPolyLine.h"
#include <deque>

#define SHOW_DEBUG_LINES 0

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
    
    //Auto backup
    NSTimer* _autoSave;
    UIAlertView* _restorSessionAlert;
}
@end

//Gestures
@interface PAMES2ViewController()
{
    //Branch creation vars
    bool isFirsPointOnAModel;
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
    RAPolyLine* polyline2;
    vector<Vec3f> polyline1Data;
    vector<Vec3f> polyline2Data;
    
    string* vShader;
    string* fShader;
    
    vector<RAPolyLine*> debugPolylines;
    
    //Undo
    std::deque<HMesh::Manifold> _undoQueue;
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
    
    if ([self backupExist]) {
        _restorSessionAlert = [[UIAlertView alloc] initWithTitle:nil message:@"Restore last session" delegate:self cancelButtonTitle:nil otherButtonTitles:@"Yes", @"No", nil];
        [_restorSessionAlert show];
    } else {
        [self startBackupTimer];
    }
    
    UIButton* undoBtn = [UIButton buttonWithType:UIButtonTypeCustom];
    [undoBtn setFrame:CGRectMake(0, self.view.frame.size.height-100, 100, 100)];
    [undoBtn addTarget:self action:@selector(undoButtonClicked:) forControlEvents:UIControlEventTouchUpInside];
    undoBtn.autoresizingMask = UIViewAutoresizingFlexibleTopMargin;
    [self.view addSubview:undoBtn];

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

//Load empty workspace
-(void)loadEmptyMeshData
{
    float a = 1;
    bounds = {Vec3f(-a,-a,-a), Vec3f(a,a,a), Vec3f(0,0,0), (2*a)*sqrtf(3.0f)/2.0f};
    GLfloat zNear = 1.0;
    viewVolumeCenter = Vec3f(0, 0, -1*(zNear + 10*bounds.radius));
    
    [self setupBoundingBox];
}

//Load initial mesh from OBJ file
-(void)loadMeshData
{
    [self setPaused:YES]; //pause rendering
    
    //Load obj file
    NSString* objPath = [[NSBundle mainBundle] pathForResource:@"HAND PAM 15" ofType:@"obj"];
    if (objPath) {
        if (pamManifold != nullptr) {
            delete pamManifold;
        }
        
        pamManifold = new PAMManifold();
        pamManifold->setupShaders();
        if (pamManifold->loadObjFile(objPath.UTF8String)) {
            pamManifold->enabled = true;
            
            bounds = pamManifold->getBoundingBox();
            
            GLfloat zNear = 1.0f;
            GLfloat newOriginZ = -1 * (zNear + bounds.radius);
            GLfloat curOriginZ = bounds.center[2];
            
            Vec3f tV = Vec3f(0, 0, newOriginZ - curOriginZ);
            pamManifold->translate(tV);
            viewVolumeCenter = tV + bounds.center;
            [self setupBoundingBox];
        }
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

#pragma mark - Gesture Recongnizers
-(void)addGestureRecognizersToView:(UIView*)view
{
    //Two finger tap
    UITapGestureRecognizer* twoFingerTap = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleTwoFingeTapGesture:)];
    twoFingerTap.numberOfTapsRequired = 1;
    twoFingerTap.numberOfTouchesRequired = 2;
    [view addGestureRecognizer:twoFingerTap];
    
    //Side Rotation
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
    
    //Long Press
    UILongPressGestureRecognizer* longPress = [[ UILongPressGestureRecognizer alloc] initWithTarget:self action:@selector(handleLongPressGesture:)];
    longPress.numberOfTouchesRequired = 1;
    [view addGestureRecognizer:longPress];
    
    //Double Tap
    UITapGestureRecognizer* doubleTap = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleDoubleTapGesture:)];
    doubleTap.numberOfTouchesRequired = 1;
    doubleTap.numberOfTapsRequired = 2;
    [view addGestureRecognizer:doubleTap];

    //Three finger pan
    UIPanGestureRecognizer* threeFingerTranslation = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handleThreeFingerPanGesture:)];
    threeFingerTranslation.minimumNumberOfTouches = 3;
    [view addGestureRecognizer:threeFingerTranslation];
    
    //Single tap
    UITapGestureRecognizer* singleTap = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(handleSingleTapGesture:)];
    singleTap.numberOfTouchesRequired = 1;
    singleTap.numberOfTapsRequired = 1;
    [singleTap requireGestureRecognizerToFail:doubleTap];
    [view addGestureRecognizer:singleTap];
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

-(void)handleLongPressGesture:(UIGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    if (PAMSettingsManager::getInstance().transform) {
        return;
    }
    
    if (sender.state == UIGestureRecognizerStateBegan)
    {
        if (pamManifold->modState == PAMManifold::Modification::PIN_POINT_SET) {
            pamManifold->deleteCurrentPinPoint();
            polyline1->enabled = false;
        } else if (pamManifold->modState == PAMManifold::Modification::NONE) {
            Vec3f modelCoord;
            if (![self modelCoordinates:modelCoord forGesture:sender]) {
                RA_LOG_WARN("Touched background");
                return;
            }
            polyline1Data.clear();
            pamManifold->createPinPoint(modelCoord, polyline1Data);
            polyline1->bufferVertexDataToGPU(polyline1Data, Vec4uc(255,0,0,255), GL_LINE_STRIP);
            polyline1->enabled = true;
        }
    }
}

-(void)handleSingleTapGesture:(UIGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    if (pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED ||
        pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED_AN_MOVED)
    {
        Vec3f modelCoord;
        if (![self modelCoordinates:modelCoord forGesture:sender]) {
            RA_LOG_WARN("Touched background");
            return;
        }

        if (!pamManifold->moveDetachedBranch(modelCoord)) {
            [self popSaved];
        }
    }
    else if (pamManifold->modState == PAMManifold::Modification::BRANCH_COPIED_BRANCH_FOR_CLONING)
    {
        Vec3f modelCoord;
        if (![self modelCoordinates:modelCoord forGesture:sender]) {
            RA_LOG_WARN("Touched background");
            return;
        }
        pamManifold->cloneBranchTo(modelCoord);
    }
}

-(void)handleTwoFingeTapGesture:(UIGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    PAMSettingsManager::getInstance().transform  = !PAMSettingsManager::getInstance().transform;
    if (PAMSettingsManager::getInstance().transform) {
        _transformModeLabel.text = @"Transform";
    } else {
        _transformModeLabel.text = @"Model";
    }
}

-(void)handleDoubleTapGesture:(UIGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    Vec3f modelCoord;
    if (![self modelCoordinates:modelCoord forGesture:sender]) {
        RA_LOG_WARN("Touched background");
        return;
    }
    int iter = PAMSettingsManager::getInstance().tapSmoothing;
    //    float wrong_radius = 3*[self touchSize];
    float radius = 3*[self touchSizeForFingerSize:8.0f];
    pamManifold->smoothAtPoint(modelCoord,radius,iter);
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
        if (pamManifold->modState == PAMManifold::Modification::PIN_POINT_SET)
        {
            if (sender.state == UIGestureRecognizerStateEnded) {
                Vec3f rayOrigin;
                if (![self rayOrigin:rayOrigin forGesture:sender]) {
                    RA_LOG_WARN("Couldn't determine touch area");
                    return;
                }
                CGPoint t = [sender translationInView:self.view];
                float swipeLength = sqrtf(powf(t.x, 2) + pow(t.y, 2));
                RA_LOG_INFO("Swipe length:%f", swipeLength);

                [self saveState];
                if (swipeLength < 160) {
                    pamManifold->detachBranch(rayOrigin);
                    polyline1->enabled = false;
                } else {
                    //TODO needs a fix
//                    pamManifold->deleteBranch(rayOrigin);
                }
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED)
        {
            if (sender.state == UIGestureRecognizerStateEnded) {
                pamManifold->attachDetachedBranch();
                polyline1->enabled = false;
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED_AN_MOVED)
        {
            if (sender.state == UIGestureRecognizerStateEnded) {
                pamManifold->attachDetachedBranch();
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::NONE)
        {
            if (sender.state == UIGestureRecognizerStateBegan)
            {
                polyline1Data.clear();
                CGPoint touchPoint = [self touchPointFromGesture:sender];
                GLubyte* pixelData = [self renderToOffscreenDepthBuffer:pamManifold];
                float depth = [self depthForPoint:touchPoint depthBuffer:pixelData];
                
                if (![self modelCoordinates:firstPoint
                              forTouchPoint:Vec3f(touchPoint.x, touchPoint.y, depth)])
                {
                    RA_LOG_WARN("Touched background");
                    return;
                }
                
                isFirsPointOnAModel = (depth < 0) ? NO : YES;
                delete[] pixelData;
            }
            
            //Add touch point to a line
            CGPoint touchPoint = [self touchPointFromGesture:sender];
            Vec3f rayOrigin;
            if (![self rayOrigin:rayOrigin forTouchPoint:touchPoint]) {
                RA_LOG_WARN("Touched background");
                return;
            }
            polyline1Data.push_back(rayOrigin);
            if (polyline1Data.size() > 1) {
                polyline1->bufferVertexDataToGPU(polyline1Data, Vec4uc(255,0,0,255), GL_LINE_STRIP);
                polyline1->enabled = true;
            }

            if (sender.state == UIGestureRecognizerStateEnded ||
                sender.state == UIGestureRecognizerStateCancelled ||
                sender.state == UIGestureRecognizerStateFailed)
            {
                polyline1->enabled = false;
                [self saveState];
                bool result = pamManifold->createBranch(polyline1Data,
                                                        firstPoint,
                                                        isFirsPointOnAModel,
                                                        PAMSettingsManager::getInstance().branchWidth);
                if (!result) {
                    [self popSaved];
                }
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
        translationManager->handlePanGesture(state, /*(1.0f/zoomManager->getScaleFactor()) * */ axis);
    }
    else
    {
        if (![self modelIsLoaded])
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
                    
//                    GLfloat zNear = 1.0;
//                    GLfloat newOriginZ = -1*(zNear + bounds.radius);
//                    GLfloat curOriginZ = bounds.center[2];
                    
                    pamManifold->translate(viewVolumeCenter - bounds.center);
                    
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
        else
        {
            CGPoint touchPoint = [self scaleTouchPoint:[sender locationInView:sender.view]
                                                inView:(GLKView*)sender.view];
            Vec3f rayStartWindow = Vec3f(touchPoint.x, touchPoint.y, 0);
            Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
            Vec3f rayOrigin;
            gluUnProjectf(rayStartWindow, projectionMatrix, viewport, rayOrigin);
            Vec3f axis(rayOrigin[0], rayOrigin[1], 0);
            axis = invert_affine(viewMatrix).mul_3D_vector(axis);

            if (pamManifold->modState == PAMManifold::Modification::PIN_POINT_SET ||
                pamManifold->modState == PAMManifold::Modification::BRANCH_TRANSLATION )
            {
                if (sender.state == UIGestureRecognizerStateBegan) {
                    Vec3f modelCoord;
                    if (![self modelCoordinates:modelCoord forGesture:sender]) {
                        RA_LOG_WARN("Touched background");
                        return;
                    }
                    pamManifold->startTranslatingBranchTree(modelCoord, axis);
                } else if (sender.state == UIGestureRecognizerStateChanged) {
                    pamManifold->continueTranslatingBranchTree(axis);
                } else if (sender.state == UIGestureRecognizerStateEnded) {
                    [self saveState];
                    pamManifold->endTranslatingBranchTree(axis);
                }
            } else if (pamManifold->modState == PAMManifold::Modification::NONE ||
                       pamManifold->modState == PAMManifold::Modification::BRANCH_POSE_TRANSLATE)
            {
                if (sender.state == UIGestureRecognizerStateBegan) {
                    Vec3f modelCoord;
                    if (![self modelCoordinates:modelCoord forGesture:sender]) {
                        RA_LOG_WARN("Touched background");
                        return;
                    }
//                    [_pMesh statePosingTranslateWithTouchPoint:modelCoord translation:translation];
                } else if (sender.state == UIGestureRecognizerStateChanged) {
//                    [_pMesh continuePosingTranslate:translation];
                } else if (sender.state == UIGestureRecognizerStateEnded) {
//                    [_pMesh endPosingTranslate:translation];
                }
            }
        }
    }
}

-(void)handleThreeFingerPanGesture:(UIPanGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    if (sender.state == UIGestureRecognizerStateEnded) {
        if (pamManifold->modState == PAMManifold::Modification::PIN_POINT_SET) {
            Vec3f rayOrigin;
            if (![self rayOrigin:rayOrigin forGesture:sender]) {
                RA_LOG_WARN("Couldn't determine touch area");
                return;
            }
            [self saveState];
            pamManifold->copyBranchToBuffer(rayOrigin);
        } else if (pamManifold->modState == PAMManifold::Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE) {
            pamManifold->attachClonedBranch();
        } else if (pamManifold->modState == PAMManifold::Modification::BRANCH_COPIED_BRANCH_FOR_CLONING) {
            pamManifold->dismissCopiedBranch();
            polyline1->enabled = false;
        }
    }
}

-(void)handleRotationGesture:(UIRotationGestureRecognizer*)sender
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    if (PAMSettingsManager::getInstance().transform)
    {
        GestureState state = [self gestureStateForRecognizer:sender];
        rotManager->handleRotationGesture(state, [sender rotation], viewVolumeCenter);
    }
    else
    {
        if (pamManifold->modState == PAMManifold::Modification::PIN_POINT_SET ||
            pamManifold->modState == PAMManifold::Modification::BRANCH_ROTATION)
        {
            if (sender.state == UIGestureRecognizerStateBegan) {
                Vec3f modelCoord;
                if (![self modelCoordinates:modelCoord forGesture:sender]) {
                    RA_LOG_WARN("Touched background");
                    return;
                }
                pamManifold->startBending(modelCoord, sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateChanged) {
                pamManifold->continueBending(sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateEnded) {
                [self saveState];
                pamManifold->endBendingWithAngle(sender.rotation);
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED ||
                 pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED_AN_MOVED ||
                 pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED_ROTATE)
        {
            if (sender.state == UIGestureRecognizerStateBegan) {
                pamManifold->startRotateDetachedBranch(sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateChanged) {
                pamManifold->continueRotateDetachedBranch(sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateEnded) {
                pamManifold->endRotateDetachedBranch(sender.rotation);
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE ||
                 pamManifold->modState == PAMManifold::Modification::BRANCH_CLONE_ROTATION)
        {
            if (sender.state == UIGestureRecognizerStateBegan) {
                pamManifold->startRotateClonedBranch(sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateChanged) {
                pamManifold->continueRotateClonedBranch(sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateEnded) {
                pamManifold->endRotateClonedBranch(sender.rotation);
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::NONE ||
                 pamManifold->modState == PAMManifold::Modification::BRANCH_POSE_ROTATE)
        {
            if (sender.state == UIGestureRecognizerStateBegan) {
                Vec3f modelCoord;
                if (![self modelCoordinates:modelCoord forGesture:sender]) {
                    RA_LOG_WARN("Touched background");
                    return;
                }
                pamManifold->startPosingRotate(modelCoord, sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateChanged) {
                pamManifold->continuePosingRotate(sender.rotation);
            } else if (sender.state == UIGestureRecognizerStateEnded) {
                [self saveState];
                pamManifold->endPosingRotate(sender.rotation);
            }
        }
    }
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
    }
    else
    {
        if (pamManifold->modState == PAMManifold::Modification::PIN_POINT_SET ||
            pamManifold->modState == PAMManifold::Modification::BRANCH_SCALING)
        {
            if (sender.state == UIGestureRecognizerStateBegan) {
                Vec3f modelCoord;
                if (![self modelCoordinates:modelCoord forGesture:sender]) {
                    RA_LOG_WARN("Touched background");
                    return;
                }
                pamManifold->startScalingBranch(modelCoord, sender.scale);
            } else if (sender.state == UIGestureRecognizerStateChanged) {
                pamManifold->continueScalingBranch(sender.scale);
            } else if (sender.state == UIGestureRecognizerStateEnded) {
                [self saveState];
                pamManifold->endScalingBranchTree(sender.scale);
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE ||
                 pamManifold->modState == PAMManifold::Modification::BRANCH_CLONE_SCALING )
        {
            if (sender.state == UIGestureRecognizerStateBegan) {
                pamManifold->startScaleClonedBranch(sender.scale);
            } else if (sender.state == UIGestureRecognizerStateChanged) {
                pamManifold->continueScaleClonedBranch(sender.scale);
            } else if (sender.state == UIGestureRecognizerStateEnded) {
                pamManifold->endScaleClonedBranch(sender.scale);
            }
        }
        else if (pamManifold->modState == PAMManifold::Modification::NONE ||
                 pamManifold->modState == PAMManifold::Modification::SCULPTING_SCALING ||
                 pamManifold->modState == PAMManifold::Modification::SCULPTING_ANISOTROPIC_SCALING ||
                 pamManifold->modState == PAMManifold::Modification::SCULPTING_BUMP_CREATION)
        {
            if (sender.state == UIGestureRecognizerStateBegan)
            {
                CGPoint touchPoint1 = [self scaleTouchPoint:[sender locationOfTouch:0 inView:(GLKView*)sender.view]
                                                     inView:(GLKView*)sender.view];
                CGPoint touchPoint2 = [self scaleTouchPoint:[sender locationOfTouch:1 inView:(GLKView*)sender.view]
                                                     inView:(GLKView*)sender.view];
                
                GLubyte* pixelData = [self renderToOffscreenDepthBuffer:pamManifold];
                float depth1 = [self depthForPoint:touchPoint1 depthBuffer:pixelData];
                float depth2 = [self depthForPoint:touchPoint2 depthBuffer:pixelData];
                
                float touchSize = [self touchSize];
                
                if (depth1 < 0 || depth2 < 0)
                {
    //                Vec3f rayOrigin1, rayOrigin2;
    //                if (![self rayOrigin:rayOrigin1 forTouchPoint:touchPoint1] ||
    //                    ![self rayOrigin:rayOrigin2 forTouchPoint:touchPoint2])
    //                {
    //                    RA_LOG_WARN("Couldn't determine touch area");
    //                    return;
    //                }
                    bool anisotropic = PAMSettingsManager::getInstance().sculptScalingType == PAMSettingsManager::ScultpScalingType::Silhouette;
                    
                    //on of the fingers touched background. Start scaling.
                    if (depth1 < 0 && depth2 < 0)
                    {
                        
                        Vec2f middlePoint = 0.5f * (Vec2f(touchPoint1.x, touchPoint1.y) +
                                                    Vec2f(touchPoint2.x, touchPoint2.y));
                        float depth = [self depthForPoint:CGPointMake(floorf(middlePoint[0]), floorf(middlePoint[1]))
                                              depthBuffer:pixelData];
                        if (depth < 0) {
                            return;
                        }
                        Vec3f model;
                        [self modelCoordinates:model forTouchPoint:Vec3f(middlePoint, depth)];
                        pamManifold->startScalingSingleRib(model, Vec3f{}, sender.scale, touchSize, anisotropic, false);
                    }
                    else if (depth1 < 0)
                    {
                        Vec3f model, model2;
                        [self modelCoordinates:model forTouchPoint:Vec3f(touchPoint2.x, touchPoint2.y, depth2)];
                        [self modelCoordinates:model2 forTouchPoint:Vec3f(touchPoint1.x, touchPoint1.y, depth2)];
                        pamManifold->startScalingSingleRib(model, model2, sender.scale, touchSize, anisotropic, true);
                    }
                    else
                    {
                        Vec3f model, model2;
                        [self modelCoordinates:model forTouchPoint:Vec3f(touchPoint1.x, touchPoint1.y, depth1)];
                        [self modelCoordinates:model2 forTouchPoint:Vec3f(touchPoint2.x, touchPoint2.y, depth1)];
                        pamManifold->startScalingSingleRib(model, model2, sender.scale, touchSize, anisotropic, true);
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
                    pamManifold->startBumpCreation(modelCoord, brushSize, sender.scale);
                }
                delete [] pixelData;
            } else if (sender.state == UIGestureRecognizerStateChanged) {
                if (pamManifold->modState == PAMManifold::Modification::SCULPTING_BUMP_CREATION) {
                    pamManifold->continueBumpCreation(sender.scale);
                } else {
                    pamManifold->changeScalingSingleRib(sender.scale);
                }
            } else if (sender.state == UIGestureRecognizerStateEnded) {
                [self saveState];
                if (pamManifold->modState == PAMManifold::Modification::SCULPTING_BUMP_CREATION) {
                    pamManifold->endBumpCreation();
                } else {
                    pamManifold->endScalingSingleRib(sender.scale);
                }
            }
        }
    }
}

-(void)undoButtonClicked:(UIButton*)btn
{
    [self undo];
}

-(void)startBackupTimer
{
    _autoSave = [NSTimer timerWithTimeInterval:30
                                        target:self
                                      selector:@selector(backupSession)
                                      userInfo:nil
                                       repeats:YES];
    [[NSRunLoop mainRunLoop] addTimer:_autoSave forMode:NSDefaultRunLoopMode];
}

-(void)stopBackupTimer
{
    [_autoSave invalidate];
}

#pragma mark - OpenGL Drawing

-(void)update
{
    GLfloat aspect = (GLfloat)_glWidth / (GLfloat)_glHeight;
    float scale = 1.0f;
//    float scale = 1.0f/zoomManager->getScaleFactor();
//    float diam = bounds.radius * 2.0f;
    float rad = bounds.radius;
    
//    GLfloat zNear = 1.0;
//    viewVolumeCenter = Vec3f(0, 0, -1*(zNear + 5*bounds.radius));
    
    GLfloat left = viewVolumeCenter[0] - rad*aspect*scale;
    GLfloat right = viewVolumeCenter[0] + rad*aspect*scale;
    GLfloat bottom = viewVolumeCenter[1] - rad*scale;
    GLfloat top = viewVolumeCenter[1] + rad*scale;

    projectionMatrix = ortho_Mat4x4f(left, right, bottom, top, 1.0f, 1.0f + 20*rad);
    viewMatrix = translationManager->getTranslationMatrix() * scaling_Mat4x4f(Vec3f(zoomManager->getScaleFactor())) * rotManager->getRotationMatrix();
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
    
//    if (boundingBox != nullptr) {
//        boundingBox->projectionMatrix = projectionMatrix;
//        boundingBox->viewMatrix = viewMatrix;
//        boundingBox->draw();
//    }
    
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
    delete [] pixelData;
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
    Vec3f rayStartWindow = Vec3f(touchPoint.x, touchPoint.y, 0.01);
    Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
    int result = gluUnProjectf(rayStartWindow, projectionMatrix*viewMatrix, viewport, rayOrigin);
    if (result == 0 ) {
        return NO;
    }
    rayDirection = Vec3f(invert_affine(rotManager->getRotationMatrix()) * Vec4f(0,0,-1,0));
    return YES;
}

-(BOOL)rayOrigin:(Vec3f&)rayOrigin forGesture:(UIGestureRecognizer*)gesture
{
    Vec3f rayDir;
    return [self rayOrigin:rayOrigin rayDirection:rayDir forGesture:gesture];
}

-(BOOL)rayOrigin:(Vec3f&)rayOrigin rayDirection:(Vec3f&)rayDirection forGesture:(UIGestureRecognizer*)gesture
{
    CGPoint touchPoint = [self scaleTouchPoint:[gesture locationInView:gesture.view] inView:(GLKView*)gesture.view];
    return [self rayOrigin:rayOrigin rayDirection:rayDirection forTouchPoint:touchPoint];
}

-(float)touchSize
{
    return [self touchSizeForFingerSize:15];
}

-(float)touchSizeForFingerSize:(float)touchSizeMM
{
    //TODO change for non-retina displays
    const float mmToPx = 2048.0f/240.0f; //2048 px for 240 mm for retina display
    float touchSizePx = touchSizeMM * mmToPx;
    
    Vec3f modelCoord, modelCoord2;
    [self rayOrigin:modelCoord forTouchPoint:CGPointMake(touchSizePx, 0)];
    [self rayOrigin:modelCoord2 forTouchPoint:CGPointMake(0, 0)];
    
    float touchSize = length(modelCoord - modelCoord2);
    return touchSize;
}

-(BOOL)modelIsLoaded
{
    return pamManifold != nullptr && pamManifold->enabled && pamManifold->no_vertices() > 0;
}

template <class C>
void clearVector(std::vector<C*>& inputvector)
{
    for (int i = 0; i < inputvector.size(); i++) {
        delete inputvector[i];
    }
    inputvector.clear();
}

//#pragma mark - Undo
-(void)saveState
{
    if (_undoQueue.size() == 5) {
        _undoQueue.pop_front();
    }
    HMesh::Manifold undoMani = *pamManifold;
    _undoQueue.push_back(undoMani);
}

-(void)popSaved
{
    if (_undoQueue.size() > 0) {
        _undoQueue.pop_front();
    }
}

-(void)undo
{
    @synchronized(self) {
        if (!_undoQueue.empty())
        {
            pamManifold->copyDataFromManifold(_undoQueue.back());
            _undoQueue.pop_back();
            polyline1->enabled = false;
        }
    }
}

#pragma mark - MFMailComposeViewControllerDelegate
- (void)mailComposeController:(MFMailComposeViewController*)controller
          didFinishWithResult:(MFMailComposeResult)result error:(NSError*)error
{
	[self dismissViewControllerAnimated:YES completion:NULL];
}

#pragma mark - UIAlertView Delegate
- (void)alertView:(UIAlertView *)alertView clickedButtonAtIndex:(NSInteger)buttonIndex {
    if (_restorSessionAlert == alertView) {
        if (buttonIndex == 0) {
            [self restoreLastSession];
        }
        [self startBackupTimer];
    }
    [alertView dismissWithClickedButtonIndex:buttonIndex animated:YES];
}

#pragma mark - SettingsViewControllerDelegate

-(void)emailObj
{
    if ([MFMailComposeViewController canSendMail]) {
        NSArray* paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString* cacheFolder = [NSString stringWithFormat:@"%@/ObjFiles", [paths objectAtIndex:0]];
        if (![[NSFileManager defaultManager] fileExistsAtPath:cacheFolder]) {
            [[NSFileManager defaultManager] createDirectoryAtPath:cacheFolder withIntermediateDirectories:YES attributes:nil error:nil];
        }
        NSString *timeStampValue = [NSString stringWithFormat:@"%ld", (long)[[NSDate date] timeIntervalSince1970]];
        NSString* filePath = [NSString stringWithFormat:@"%@/obj_%@.obj", cacheFolder, timeStampValue];

        if (obj_save(filePath.UTF8String, *pamManifold))
        {
            MFMailComposeViewController *picker = [[MFMailComposeViewController alloc] init];
            picker.mailComposeDelegate = self;
            [picker setSubject:@"PAM obj"];
            [picker setToRecipients:@[@"rindopuz23@gmail.com"]];
            // Attach an image to the email
            NSData *myData = [NSData dataWithContentsOfFile:filePath];
            if (myData != nil) {
                
                [picker addAttachmentData:myData mimeType:@"text/plain" fileName:@"PAM.obj"];
                [self presentViewController:picker animated:YES completion:nil];
            } else {
                [[[UIAlertView alloc] initWithTitle:@"Error" message:@"Could not retrieve obj file" delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil] show];
            }
        }
    } else {
        [[[UIAlertView alloc] initWithTitle:@"Error" message:@"No email account is setup" delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil] show];
    }
}

-(void)subdivide
{
    [self saveState];
    pamManifold->subdivide();
}

-(void)loadArmadillo
{
    [self resetTransformations];
    [self loadMeshData];
}

-(void)showRibJunctions {
//    [_pMesh showRibJunctions];
}

-(void)globalSmoothing
{
    [self saveState];
    pamManifold->globalSmoothing();
}

-(void)showSkeleton:(BOOL)show
{
//    PAMSettingsManager::getInstance().showSkeleton = show;
//    pamManifold->showSkeleton(show);
}

-(void)clearModel
{
    pamManifold->clearVertexData();
    [self loadEmptyMeshData];
    [self resetTransformations];
    PAMSettingsManager::getInstance().transform = false;
    _transformModeLabel.text = @"Model";
}

-(void)smoothingBrushSize:(float)brushSize
{
    PAMSettingsManager::getInstance().smoothingBrushSize = brushSize;
}

-(void)baseSmoothingIterations:(float)iter
{
    PAMSettingsManager::getInstance().baseSmoothingIterations = iter;
}

-(void)branchWidth:(float)width
{
    PAMSettingsManager::getInstance().branchWidth = width;
}

-(void)tapSmoothing:(float)power
{
    PAMSettingsManager::getInstance().tapSmoothing = power;
}

-(void)spineSmoothing:(BOOL)spineSmoothin
{
    PAMSettingsManager::getInstance().spineSmoothing = spineSmoothin;
}

-(void)poleSmoothing:(BOOL)poleSmoothing
{
    PAMSettingsManager::getInstance().poleSmoothing = poleSmoothing;
}

-(void)resetTransformations
{
    rotManager->reset();
    translationManager->reset();
    zoomManager->reset();
}

//SCULPTING
-(void)scalingSculptTypeChanged:(PAMSettingsManager::ScultpScalingType)type
{
    PAMSettingsManager::getInstance().sculptScalingType = type;
}

-(void)silhouetteScalingBrushSize:(float)width
{
    PAMSettingsManager::getInstance().silhouetteScalingBrushSize = width;
}

#pragma mark - Save/Restore modeling session
-(void)backupSession
{
    if (![self modelIsLoaded]) {
        return;
    }
    
    dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0ul);
    dispatch_async(queue, ^{
        NSArray* paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString* cacheFolder = [NSString stringWithFormat:@"%@/Backups", [paths objectAtIndex:0]];
        if (![[NSFileManager defaultManager] fileExistsAtPath:cacheFolder]) {
            [[NSFileManager defaultManager] createDirectoryAtPath:cacheFolder withIntermediateDirectories:YES attributes:nil error:nil];
        }
        //        NSString *timeStampValue = [NSString stringWithFormat:@"%ld", (long)[[NSDate date] timeIntervalSince1970]];
        NSString* filePath = [NSString stringWithFormat:@"%@/backup.obj", cacheFolder];
        
        
        if (pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED ||
            pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED_AN_MOVED ||
            pamManifold->modState == PAMManifold::Modification::BRANCH_DETACHED_ROTATE ||
            pamManifold->modState == PAMManifold::Modification::BRANCH_COPIED_AND_MOVED_THE_CLONE ||
            pamManifold->modState == PAMManifold::Modification::BRANCH_CLONE_ROTATION ||
            pamManifold->modState == PAMManifold::Modification::BRANCH_CLONE_SCALING)
        {
            return;
        }

        bool saved = obj_save(filePath.UTF8String, *pamManifold);
        
        dispatch_sync(dispatch_get_main_queue(), ^{
            if (saved) {
                RA_LOG_INFO("Session saved");
            } else {
                RA_LOG_INFO("Failed to save session");
            }
        });
    });
}

-(void)restoreLastSession
{
    NSArray* paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString* cacheFolder = [NSString stringWithFormat:@"%@/Backups", [paths objectAtIndex:0]];
    if ([[NSFileManager defaultManager] fileExistsAtPath:cacheFolder]) {
        
        [self setPaused:YES]; //pause rendering
        
        NSFileManager* fileManager = [NSFileManager defaultManager];
        NSArray* allBackups = [fileManager contentsOfDirectoryAtPath:cacheFolder error:nil];
        NSArray* sortedBackups = [allBackups sortedArrayUsingSelector:@selector(localizedCaseInsensitiveCompare:)];
        NSString* lastPath = [NSString stringWithFormat:@"%@/%@",cacheFolder, [sortedBackups lastObject]];
        
        //Reset all transformations. Remove all previous screws and plates
        [self resetTransformations];
        
        if (pamManifold != nullptr) {
            delete pamManifold;
        }
        
        pamManifold = new PAMManifold();
        pamManifold->setupShaders();
        if (pamManifold->loadObjFile(lastPath.UTF8String))
        {
            pamManifold->enabled = true;

            float a = 1;
            bounds = {Vec3f(-a,-a,-a), Vec3f(a,a,a), Vec3f(0,0,0), (2*a)*sqrtf(3.0f)/2.0f};
            GLfloat zNear = 1.0;
            viewVolumeCenter = Vec3f(0, 0, -1*(zNear + 10*bounds.radius));
            
            pamManifold->translate(viewVolumeCenter - bounds.center);

            [self setupBoundingBox];
        }
        
        //Load obj file
        [self setPaused:NO];
    }
}

-(BOOL)backupExist {
    NSArray* paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString* cacheFolder = [NSString stringWithFormat:@"%@/Backups", [paths objectAtIndex:0]];
    if (![[NSFileManager defaultManager] fileExistsAtPath:cacheFolder]) {
        return NO;
    } else {
        return YES;
    }
}


@end
