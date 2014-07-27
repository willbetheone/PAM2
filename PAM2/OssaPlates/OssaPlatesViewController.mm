//
//  OssaPlatesViewController.m
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-07-08.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#import "OssaPlatesViewController.h"

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
#include "RAPolylineUtilities.h"
#include "RAPolyLine.h"
#include "SegmentSpline.h"
#include "Plate.h"

using namespace std;
using namespace CGLA;
using namespace PAMMesh;
using namespace RAEngine;
using namespace Ossa;

@interface OssaPlatesViewController ()
{
    //offscreen color and depth buffer
    RAES2OffScreenBuffer* offScreenBuffer;
    
    Mat4x4f viewMatrix;
    Mat4x4f projectionMatrix;
    
    PAMManifold* pamManifold;
    RABoundingBox* boundingBox;
    Bounds bounds;
    Vec3f viewVolumeCenter;
    
    RARotationManager* rotManager;
    RAZoomManager* zoomManager;
    RATranslationManager* translationManager;
    
    UIButton* drawLine;
    BOOL drawLineMode;
    
    vector<Vec3f> polyline;
    RAPolyLine* userDrawnCurve;
    RAPolyLine* userDrawnCurveTesselated;
    RAPolyLine* plateSpline;
    
    GLubyte* tempDepthBuffer;
    vector<RAUnitSphere*> unitSpheres;
    Plate* plate;

    SegmentSpline* segSpline;
    RAUnitSphere* movedUnitSphere;
    int movedUnitSphereIndex;
    std::string vShader_Cplus;
    std::string fShader_Cplus;
}
@end

@implementation OssaPlatesViewController

- (id)init
{
    self = [super init];
    if (self) {
        // Custom initialization
        rotManager = new RARotationManager();
        zoomManager = new RAZoomManager();
        translationManager = new RATranslationManager();
    }
    return self;
}

#pragma mark - View cycle and OpenGL setup
- (void)viewDidLoad
{
    [super viewDidLoad];
    [self setupGL];
    [self addGestureRecognizersToView:self.view];
    
    drawLine = [UIButton buttonWithType:UIButtonTypeCustom];
    [drawLine setTitle:@"Draw Line" forState:UIControlStateNormal];
    [drawLine setTitleColor:[UIColor blackColor] forState:UIControlStateNormal];
    [drawLine setFrame:CGRectMake(10, 10, 150, 40)];
    [drawLine addTarget:self action:@selector(drawLineButtonClicked:) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:drawLine];
    
    drawLineMode = NO;
    vShader_Cplus = std::string([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"vsh"].UTF8String);
    fShader_Cplus = std::string([[NSBundle mainBundle] pathForResource:@"PosColorShader" ofType:@"fsh"].UTF8String);

    userDrawnCurve = new RAPolyLine();
    userDrawnCurve->setupShaders(vShader_Cplus, fShader_Cplus);
    
    userDrawnCurveTesselated = new RAPolyLine();
    userDrawnCurveTesselated->setupShaders(vShader_Cplus, fShader_Cplus);
    
    plateSpline = new RAPolyLine();
    plateSpline->setupShaders(vShader_Cplus, fShader_Cplus);
    
    unitSpheres = vector<RAUnitSphere*>();
    plate = nullptr;
    
    [self loadMeshData];
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
    
    ((GLKView *)self.view).context = nil;
    [EAGLContext setCurrentContext:nil];
}

-(void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    offScreenBuffer = new RAES2OffScreenBuffer();
    offScreenBuffer->createOffScreenBuffer(_glWidth, _glHeight);
}

-(void)setupGL
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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
    Vec3f toucP{};
    [self modelCoordinates:toucP forGesture:sender];
}

-(void)handleOneFingerPanGesture:(UIPanGestureRecognizer*)sender
{
    if (drawLineMode) {
        [self handlePlateCreation:sender];
    } else {
        [self handleControlPointMovement:sender];
    }
}

-(void)handleControlPointMovement:(UIGestureRecognizer*) sender {
    CGPoint cocoaTouch = [sender locationInView:sender.view];
    GestureState state = [self gestureStateForRecognizer:sender];
    
    if (sender.state == UIGestureRecognizerStateBegan ) {
        Vec3f rayOrigin, rayDirection;
        [self rayOrigin:rayOrigin rayDirection:rayDirection forGesture:sender];
        movedUnitSphere = nullptr;
        int i = 0;
        for (RAUnitSphere* uSphere : unitSpheres) {
            if (uSphere->testBoundingBoxIntersection(rayOrigin, rayDirection, 0, FLT_MAX)) {
                RA_LOG_INFO("hit uSpher");
                movedUnitSphere = uSphere;
                movedUnitSphereIndex = i;
                tempDepthBuffer = [self renderToOffscreenDepthBuffer:pamManifold];
                return;
            }
            i++;
        }
    } else if (sender.state == UIGestureRecognizerStateChanged) {
        if (movedUnitSphere != nullptr) {
            Vec3f modelCoord;
            CGPoint glTouch = [self scaleTouchPoint:cocoaTouch inView:(GLKView*)self.view];
            
            if ([self modelCoordinates:modelCoord forTouchPoint:glTouch depthBuffer:tempDepthBuffer]) {
                movedUnitSphere->setCenter(modelCoord);
                segSpline->setControlPoint(movedUnitSphereIndex, modelCoord);
                vector<Vec3f> splineData;
                segSpline->getSpline(splineData);
                
                vector<Vec3f> centerSplineData;
                vector<Vec3f> normSplineData;
                vector<Vec3f> tangentSplineData;
//                centers(centerSplineData, splineData);
//                tangents(tangentSplineData, splineData);
                assert(centerSplineData.size() == tangentSplineData.size());
                
                for (Vec3f point : centerSplineData) {
                    Vec3f norm;
                    if (pamManifold->normal(point, norm)) {
                        normSplineData.push_back(normalize(norm));
                    } else {
                        RA_LOG_WARN("couldnt find norm vector");
                        return;
                    }
                }
                
                plate->setSpline(centerSplineData, normSplineData, tangentSplineData);
            } else {
                RA_LOG_ERROR("Couldnt get the correct model coord for touch point");
            }
            return;
        }
    } else {
        if (movedUnitSphere != nullptr) {
            movedUnitSphere = nullptr;
            delete tempDepthBuffer;
            return;
        }
    }
    
    Vec2i glTouch = Vec2i(cocoaTouch.x, [sender.view bounds].size.height - cocoaTouch.y);
    rotManager->handlePanGesture(state, glTouch, viewVolumeCenter);
}

-(void)handlePlateCreation:(UIGestureRecognizer*) sender
{
    CGPoint cocoaTouch = [sender locationInView:sender.view];
    GestureState state = [self gestureStateForRecognizer:sender];
    CGPoint glTouch = [self scaleTouchPoint:cocoaTouch inView:(GLKView*)self.view];
    
    if (state == GestureState::Began)
    {
        polyline.clear();
        tempDepthBuffer = [self renderToOffscreenDepthBuffer:pamManifold];
        Vec3f modelCoord;
        if ([self modelCoordinates:modelCoord forTouchPoint:glTouch depthBuffer:tempDepthBuffer]) {
            polyline.push_back(modelCoord);
            userDrawnCurve->bufferVertexDataToGPU(polyline, Vec4uc(255,0,0,255), GL_LINE_STRIP);
        } else {
            RA_LOG_ERROR("Couldnt get the correct model coord for touch point");
        }
    }
    else if (state == GestureState::Changed)
    {
        Vec3f modelCoord;
        if ([self modelCoordinates:modelCoord forTouchPoint:glTouch depthBuffer:tempDepthBuffer]) {
            polyline.push_back(modelCoord);
            userDrawnCurve->bufferVertexDataToGPU(polyline, Vec4uc(255,0,0,255), GL_LINE_STRIP);
        } else {
            RA_LOG_ERROR("Couldnt get the correct model coord for touch point");
        }
    }
    else if (state == GestureState::Ended)
    {
        Vec3f modelCoord;
        if ([self modelCoordinates:modelCoord forTouchPoint:glTouch depthBuffer:tempDepthBuffer]) {
            polyline.push_back(modelCoord);
            userDrawnCurve->bufferVertexDataToGPU(polyline, Vec4uc(255,0,0,255), GL_LINE_STRIP);
        } else {
            RA_LOG_ERROR("Couldnt get the correct model coord for touch point");
        }
        
        if (segSpline != nullptr) {
            Vec3f firstPoint = polyline[0];
            for (int i = 0; i < segSpline->getControlQuantity(); i++) {
                if (segSpline->isCloseToControlPoint(firstPoint, i)) {
                    [self extenedPlate:i];
                    return;
                }
            }
        }
        
        [self createNewPlate];
    }
}

-(void)extenedPlate:(int)cntPoint {
    
    vector<Vec3f> splineControlPoints;
    vector<Vec3f> splineData;
    polyline.insert(polyline.begin(), segSpline->getControlPoint(cntPoint));
    segSpline->setSampleData(polyline, cntPoint);
    segSpline->getControlPoints(splineControlPoints);
    segSpline->getSpline(splineData);
    userDrawnCurveTesselated->bufferVertexDataToGPU(splineControlPoints, Vec4uc(0,255,0,255), GL_LINE_STRIP);
    plateSpline->bufferVertexDataToGPU(splineData, Vec4uc(0,0,255,255), GL_LINE_STRIP);
    
    vector<Vec3f> centerSplineData;
    vector<Vec3f> normSplineData;
    vector<Vec3f> tangentSplineData;
//    centers(centerSplineData, splineData);
//    tangents(tangentSplineData, splineData);
    assert(centerSplineData.size() == tangentSplineData.size());
    
    for (Vec3f point : centerSplineData) {
        Vec3f norm;
        if (pamManifold->normal(point, norm)) {
            normSplineData.push_back(normalize(norm));
        } else {
            RA_LOG_WARN("couldnt find norm vector");
            return;
        }
    }

    plate->setSpline(centerSplineData, normSplineData, tangentSplineData);
    
    clearVector(unitSpheres);
    for (int i = 0; i < segSpline->getControlQuantity(); i++)
    {
        Vec3f ctrPnt = segSpline->getControlPoint(i);
        RAUnitSphere* unitSphere = new RAUnitSphere(Vec3f(ctrPnt[0],ctrPnt[1],ctrPnt[2]));
        unitSphere->setupShaders(vShader_Cplus, fShader_Cplus);
        unitSphere->loadObjFile([[NSBundle mainBundle] pathForResource:@"sphere" ofType:@"obj"].UTF8String);
        unitSphere->scale(Vec3(0.02*bounds.radius, 0.02*bounds.radius, 0.02*bounds.radius), Vec3(0,0,0));
        unitSpheres.push_back(unitSphere);
    }
    
    delete tempDepthBuffer;
}

-(void)createNewPlate {
    clearVector(unitSpheres);
    delete segSpline;
    
    segSpline =  new SegmentSpline(polyline, 0.1f);
    vector<Vec3f> splineControlPoints;
    vector<Vec3f> splineData;
    segSpline->getControlPoints(splineControlPoints);
    segSpline->getSpline(splineData);
    userDrawnCurveTesselated->bufferVertexDataToGPU(splineControlPoints, Vec4uc(0,255,0,255), GL_LINE_STRIP);
    plateSpline->bufferVertexDataToGPU(splineData, Vec4uc(0,0,255,255), GL_LINE_STRIP);
    
    vector<Vec3f> centerSplineData;
    vector<Vec3f> normSplineData;
    vector<Vec3f> tangentSplineData;
//    centers(centerSplineData, splineData);
//    tangents(tangentSplineData, splineData);
    assert(centerSplineData.size() == tangentSplineData.size());
    
    for (Vec3f point : centerSplineData) {
        
        Vec3f norm;
        if (pamManifold->normal(point, norm)) {
            normSplineData.push_back(normalize(norm));
        } else {
            RA_LOG_WARN("couldnt find norm vector");
            return;
        }
    }
    
    plate = new Plate();
    plate->setSpline(centerSplineData, normSplineData, tangentSplineData);
    plate->setupShaders(vShader_Cplus, fShader_Cplus);
    plate->loadObjFile([[NSBundle mainBundle] pathForResource:@"roundsegment" ofType:@"obj"].UTF8String);
    plate->scale = 0.1f/2.0;
    
    for (int i = 0; i < segSpline->getControlQuantity(); i++)
    {
        Vec3f ctrPnt = segSpline->getControlPoint(i);
        RAUnitSphere* unitSphere = new RAUnitSphere(Vec3f(ctrPnt[0],ctrPnt[1],ctrPnt[2]));
        unitSphere->setupShaders(vShader_Cplus, fShader_Cplus);
        unitSphere->loadObjFile([[NSBundle mainBundle] pathForResource:@"sphere" ofType:@"obj"].UTF8String);
        unitSphere->scale(Vec3(0.02*bounds.radius, 0.02*bounds.radius, 0.02*bounds.radius), Vec3(0,0,0));
        unitSpheres.push_back(unitSphere);
    }
    
    delete tempDepthBuffer;
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
    
//    if (state == GestureState::Began) {
//        CGPoint touchPoint = [self scaleTouchPoint:[sender locationInView:sender.view]
//                                            inView:(GLKView*)sender.view];
//        Vec3f rayStartWindow = Vec3f(touchPoint.x, touchPoint.y, 0);
//        Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
//        Vec3f rayOrigin;
//        gluUnProjectf(rayStartWindow, projectionMatrix, viewport, rayOrigin);
//
//        Vec3f view = viewMatrix.mul_3D_point(viewVolumeCenter);
//        Vec3f axis(rayOrigin[0] - view[0], rayOrigin[1] - view[1], 0);
//y
//        translationManager->handlePanGesture(state, axis);
//    } else if (state == GestureState::Changed) {
//        Vec3f newPoint = scaling_Mat3x3f(Vec3f(zoomManager->getScaleFactor(), zoomManager->getScaleFactor(), 1)) * translationManager->startPoint;
//        Vec3f trans = newPoint - translationManager->startPoint;
//        translationManager->handlePanGesture(state, trans);
//    } else if (state == GestureState::Ended) {
//        
//    }
}

-(void)handleTwoFingerPanGesture:(UIPanGestureRecognizer*)sender
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

-(void)drawLineButtonClicked:(id)selector
{
    drawLineMode = !drawLineMode;
    if (drawLineMode) {
        glEnable (GL_BLEND);
    } else {
        glDisable(GL_BLEND);
    }
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
        pamManifold->buildKDTree();
        bounds = pamManifold->getBoundingBox();
        
        float rad = bounds.radius;
        GLfloat zNear = 1.0;
        
        GLfloat newOriginZ = -1*(zNear + rad);
        GLfloat curOriginZ = bounds.center[2];
        
        pamManifold->translate(Vec3f(0, 0, newOriginZ - curOriginZ));
        viewVolumeCenter = pamManifold->getModelMatrix() * Vec4f(bounds.center);
        
        boundingBox = new RABoundingBox(bounds.minBound, bounds.maxBound);
        boundingBox->setupShaders(vShader_Cplus, fShader_Cplus);
        boundingBox->bufferVertexDataToGPU();
        boundingBox->translate(viewVolumeCenter - bounds.center);
    }
    
    [self setPaused:NO];
}

#pragma mark - OpenGL Drawing

-(void)update
{
    float diam = bounds.radius * 2.0f;
    GLfloat aspect = (GLfloat)_glWidth / (GLfloat)_glHeight;
    float scale = 1.0f/zoomManager->getScaleFactor();
    
    GLfloat left = viewVolumeCenter[0] - diam*aspect*scale;
    GLfloat right = viewVolumeCenter[0] + diam*aspect*scale;
    GLfloat bottom = viewVolumeCenter[1] - diam*scale;
    GLfloat top = viewVolumeCenter[1] + diam*scale;
    
    projectionMatrix = ortho_Mat4x4f(left, right, bottom, top, 1.0f, 1.0f + diam);
    viewMatrix = translationManager->getTranslationMatrix() * rotManager->getRotationMatrix();
}

-(void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    if (drawLineMode) {
        glDisable(GL_DEPTH_TEST);
    }

    pamManifold->projectionMatrix = projectionMatrix;
    pamManifold->viewMatrix = viewMatrix;
    pamManifold->draw();
    
//    boundingBox->projectionMatrix = projectionMatrix;
//    boundingBox->viewMatrix = viewMatrix;
//    boundingBox->draw();
    
    if (drawLineMode) {
       glEnable(GL_DEPTH_TEST);
    }
    
    for (RAUnitSphere* unitSphere: unitSpheres) {
        unitSphere->projectionMatrix = projectionMatrix;
        unitSphere->viewMatrix = viewMatrix;
        unitSphere->draw();
    }

    if (plate!=nullptr) {
        plate->projectionMatrix = projectionMatrix;
        plate->viewMatrix = viewMatrix;
        plate->draw();
    }
    
    userDrawnCurve->projectionMatrix = projectionMatrix;
    userDrawnCurve->viewMatrix = viewMatrix;
    userDrawnCurve->draw();
    
//    userDrawnCurveTesselated->projectionMatrix = projectionMatrix;
//    userDrawnCurveTesselated->viewMatrix = viewMatrix;
//    userDrawnCurveTesselated->draw();
    
//    plateSpline->projectionMatrix = projectionMatrix;
//    plateSpline->viewMatrix = viewMatrix;
//    plateSpline->draw();

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

-(BOOL)rayOrigin:(Vec3f&)rayOrigin rayDirection:(Vec3f&)rayDirection forTouchPoint:(CGPoint)touchPoint
{
    Vec3f rayStartWindow = Vec3f(touchPoint.x, touchPoint.y, 0);
    Vec4f viewport = Vec4f(0, 0, _glWidth, _glHeight);
    int result = gluUnProjectf(rayStartWindow, projectionMatrix*viewMatrix, viewport, rayOrigin);
    if (result == 0 ) {
        return NO;
    }
    rayDirection = Vec3f(invert_ortho(rotManager->getRotationMatrix()) * Vec4f(0,0,-1,0));
    return YES;
}

-(BOOL)rayOrigin:(Vec3f&)rayOrigin rayDirection:(Vec3f&)rayDirection forGesture:(UIGestureRecognizer*)gesture
{
    CGPoint touchPoint = [self scaleTouchPoint:[gesture locationInView:gesture.view] inView:(GLKView*)gesture.view];
    return [self rayOrigin:rayOrigin rayDirection:rayDirection forTouchPoint:touchPoint];
}

template <class C>
void clearVector(std::vector<C*>& inputvector)
{
    for (int i = 0; i < inputvector.size(); i++) {
        delete inputvector[i];
    }
    inputvector.clear();
}

//-(void)clearPlateSphereVector:(std::vector<RAPlateSegment*>) vector
//{
//    for (auto v: vector) {
//        delete v;
//    }
//}


@end
