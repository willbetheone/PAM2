//
//  SettingsViewController.m
//  PAM
//
//  Created by Rinat Abdrashitov on 12/25/2013.
//  Copyright (c) 2013 Rinat Abdrashitov. All rights reserved.
//

#import "SettingsViewController.h"

using namespace PAMMesh;

@interface SettingsViewController ()

@end

@implementation SettingsViewController

- (id)init
{
    self = [super init];
    if (self) {
        // Custom initialization

    }
    return self;
}

-(void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    contentView = [[UIScrollView alloc] initWithFrame:self.view.bounds];
    contentView.autoresizingMask=UIViewAutoresizingFlexibleHeight | UIViewAutoresizingFlexibleWidth;

    [self.view addSubview:contentView];
    self.view.backgroundColor = [UIColor whiteColor];
    contentView.backgroundColor = [UIColor whiteColor];
    
    int nextY = 10;
//    UILabel* skeletonLabel = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY, 100, 30)];
//    [skeletonLabel setText:@"Skeleton"];
//    [contentView addSubview:skeletonLabel];
//    _skeletonSwitch = [[UISwitch alloc] initWithFrame:CGRectMake(145, nextY, 30, 20)];
//    [_skeletonSwitch addTarget:self action:@selector(skeletonSwitchClicked:) forControlEvents:UIControlEventValueChanged];
//    [contentView addSubview:_skeletonSwitch];
    
//    nextY = CGRectGetMaxY(skeletonLabel.frame);
    _clearModelBtn = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_clearModelBtn setFrame:CGRectMake(15, nextY + 10, 100, 30)];
    [_clearModelBtn setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_clearModelBtn addTarget:self action:@selector(clearModelButton:) forControlEvents:UIControlEventTouchUpInside];
    [_clearModelBtn setTitle:@"Clear Model" forState:UIControlStateNormal];
    [contentView addSubview:_clearModelBtn];
    
    nextY = CGRectGetMaxY(_clearModelBtn.frame);
    _resetBtn = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_resetBtn setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_resetBtn setTitle:@"Reset Transformations" forState:UIControlStateNormal];
    [_resetBtn addTarget:self action:@selector(resetButton:) forControlEvents:UIControlEventTouchUpInside];
    [_resetBtn setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_resetBtn];
    
//    nextY = CGRectGetMaxY(_resetBtn.frame);
//    _showRibJunctionsBtn = [UIButton buttonWithType:UIButtonTypeRoundedRect];
//    [_showRibJunctionsBtn setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
//    [_showRibJunctionsBtn setTitle:@"Show Rib Junctions" forState:UIControlStateNormal];
//    [_showRibJunctionsBtn addTarget:self action:@selector(showRibJunctions:) forControlEvents:UIControlEventTouchUpInside];
//    [_showRibJunctionsBtn setFrame:CGRectMake(15, nextY + 10, 200, 30)];
//    [contentView addSubview:_showRibJunctionsBtn];
    
    nextY = CGRectGetMaxY(_resetBtn.frame);
    _globalSmoothingBtn = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_globalSmoothingBtn setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_globalSmoothingBtn setTitle:@"Smooth Model" forState:UIControlStateNormal];
    [_globalSmoothingBtn addTarget:self action:@selector(globalSmoothing:) forControlEvents:UIControlEventTouchUpInside];
    [_globalSmoothingBtn setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_globalSmoothingBtn];
    
    nextY = CGRectGetMaxY(_globalSmoothingBtn.frame);
    _subdivide = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_subdivide setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_subdivide setTitle:@"Subdivide Model" forState:UIControlStateNormal];
    [_subdivide addTarget:self action:@selector(subdivide:) forControlEvents:UIControlEventTouchUpInside];
    [_subdivide setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_subdivide];
    
    nextY = CGRectGetMaxY(_subdivide.frame);
    _loadArmadillo = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_loadArmadillo setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_loadArmadillo setTitle:@"Load Sample Model" forState:UIControlStateNormal];
    [_loadArmadillo addTarget:self action:@selector(loadArmadillo:) forControlEvents:UIControlEventTouchUpInside];
    [_loadArmadillo setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_loadArmadillo];
    
    nextY = CGRectGetMaxY(_loadArmadillo.frame);
    _saveObjFile = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_saveObjFile setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_saveObjFile setTitle:@"Email OBJ" forState:UIControlStateNormal];
    [_saveObjFile addTarget:self action:@selector(emailObj:) forControlEvents:UIControlEventTouchUpInside];
    [_saveObjFile setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_saveObjFile];
    
    /*
     * BRANCH CREATION
     */
    
    nextY = CGRectGetMaxY(_saveObjFile.frame);
    UILabel* branchCreationHeader = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY + 10, 300, 30)];
    [branchCreationHeader setText:@"BRANCH CREATION SETTINGS"];
    branchCreationHeader.font = [UIFont boldSystemFontOfSize:15.0f];
    branchCreationHeader.adjustsFontSizeToFitWidth = YES;
    branchCreationHeader.textAlignment = NSTextAlignmentCenter;
    [contentView addSubview:branchCreationHeader];

    /******/
    nextY = CGRectGetMaxY(branchCreationHeader.frame);
    UILabel* spineSmoothinLabel = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY, 200, 30)];
    [spineSmoothinLabel setText:@"Spine smoothing"];
    [contentView addSubview:spineSmoothinLabel];
    
    nextY = CGRectGetMaxY(branchCreationHeader.frame);
    _spineSmoothing = [[UISwitch alloc] initWithFrame:CGRectMake(200, nextY, 30, 20)];
    [_spineSmoothing addTarget:self action:@selector(spineSmoothing:) forControlEvents:UIControlEventValueChanged];
    [contentView addSubview:_spineSmoothing];
    
    /******/
    nextY = CGRectGetMaxY(_spineSmoothing.frame) + 10;
    UILabel* poleSmoothing = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY, 200, 30)];
    [poleSmoothing setText:@"Smooth Poles"];
    [contentView addSubview:poleSmoothing];
    
    nextY = CGRectGetMaxY(_spineSmoothing.frame) + 10;
    _poleSmoothing = [[UISwitch alloc] initWithFrame:CGRectMake(200, nextY, 30, 20)];
    [_poleSmoothing addTarget:self action:@selector(poleSmoothing:) forControlEvents:UIControlEventValueChanged];
    [contentView addSubview:_poleSmoothing];
    
    /*****/
    nextY = CGRectGetMaxY(_poleSmoothing.frame);
    UILabel* smoothingBrushSize = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY + 10, 300, 30)];
    [smoothingBrushSize setText:@"Base smoothing multiplier of radius 0.5-2"];
    smoothingBrushSize.adjustsFontSizeToFitWidth = YES;
    [contentView addSubview:smoothingBrushSize];
    
    nextY = CGRectGetMaxY(smoothingBrushSize.frame);
    _smoothingSlider = [[UISlider alloc] init];
    _smoothingSlider.minimumValue = 0.5;
    _smoothingSlider.maximumValue = 2.0;
    [_smoothingSlider addTarget:self action:@selector(smoothingBrushSize:) forControlEvents:UIControlEventValueChanged];
    [_smoothingSlider setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_smoothingSlider];
    
    nextY = CGRectGetMaxY(_smoothingSlider.frame);
    UILabel* baseSmoothingIterations = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY + 10, 300, 30)];
    [baseSmoothingIterations setText:@"Base smoothing iterations 0-30"];
    baseSmoothingIterations.adjustsFontSizeToFitWidth = YES;
    [contentView addSubview:baseSmoothingIterations];
    
    nextY = CGRectGetMaxY(baseSmoothingIterations.frame);
    _baseSmoothingIterationsSlider = [[UISlider alloc] init];
    _baseSmoothingIterationsSlider.minimumValue = 0;
    _baseSmoothingIterationsSlider.maximumValue = 30;
    [_baseSmoothingIterationsSlider addTarget:self action:@selector(baseSmoothingIterations:) forControlEvents:UIControlEventValueChanged];
    [_baseSmoothingIterationsSlider setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_baseSmoothingIterationsSlider];
    
    nextY = CGRectGetMaxY(_baseSmoothingIterationsSlider.frame);
    UILabel* branchWidthSizeLabel = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY + 10, 300, 30)];
    [branchWidthSizeLabel setText:@"Branch width 0-100 degress"];
    branchWidthSizeLabel.adjustsFontSizeToFitWidth = YES;
    [contentView addSubview:branchWidthSizeLabel];
    
    nextY = CGRectGetMaxY(branchWidthSizeLabel.frame);
    _branchWidth = [[UISlider alloc] init];
    _branchWidth.minimumValue = 0;
    _branchWidth.maximumValue = 100;
    [_branchWidth addTarget:self action:@selector(branchWidth:) forControlEvents:UIControlEventValueChanged];
    [_branchWidth setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_branchWidth];
    
    /*
     * TAP SMOOTHING
     */
    
    nextY = CGRectGetMaxY(_branchWidth.frame);
    UILabel* tapSmoothingLabel = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY + 10, 300, 30)];
    [tapSmoothingLabel setText:@"Tap Smoothing"];
    tapSmoothingLabel.adjustsFontSizeToFitWidth = YES;
    [contentView addSubview:tapSmoothingLabel];
    
    nextY = CGRectGetMaxY(tapSmoothingLabel.frame);
    _tapSmoothing = [[UISlider alloc] init];
    _tapSmoothing.minimumValue = 1;
    _tapSmoothing.maximumValue = 8;
    [_tapSmoothing addTarget:self action:@selector(tapSmoothing:) forControlEvents:UIControlEventValueChanged];
    [_tapSmoothing setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_tapSmoothing];
    
    /*
     * SCULPTING
     */
    
    nextY = CGRectGetMaxY(_tapSmoothing.frame);
    UILabel* suclptingHeader = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY + 10, 300, 30)];
    [suclptingHeader setText:@"SCULPTING SETTINGS"];
    suclptingHeader.font = [UIFont boldSystemFontOfSize:15.0f];
    suclptingHeader.adjustsFontSizeToFitWidth = YES;
    suclptingHeader.textAlignment = NSTextAlignmentCenter;
    [contentView addSubview:suclptingHeader];
    
//    nextY = CGRectGetMaxY(suclptingHeader.frame);
//    UILabel* silhouetteScalingBrushSizeLabel = [[UILabel alloc] initWithFrame:CGRectMake(15, nextY + 10, 300, 30)];
//    [silhouetteScalingBrushSizeLabel setText:@"Silhouette scalling arc 0-180 degress"];
//    silhouetteScalingBrushSizeLabel.adjustsFontSizeToFitWidth = YES;
//    [contentView addSubview:silhouetteScalingBrushSizeLabel];
    
//    nextY = CGRectGetMaxY(silhouetteScalingBrushSizeLabel.frame);
//    _silhouetteScalingBrushSize = [[UISlider alloc] init];
//    _silhouetteScalingBrushSize.minimumValue = 0;
//    _silhouetteScalingBrushSize.maximumValue = 100;
//    [_silhouetteScalingBrushSize addTarget:self action:@selector(silhouetteScalingBrushSize:) forControlEvents:UIControlEventValueChanged];
//    [_silhouetteScalingBrushSize setFrame:CGRectMake(15, nextY + 10, 200, 30)];
//    [contentView addSubview:_silhouetteScalingBrushSize];    
    
    nextY = CGRectGetMaxY(suclptingHeader.frame);
    _circularScalingSculpt = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_circularScalingSculpt setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_circularScalingSculpt setTitle:@"Circular scaling" forState:UIControlStateNormal];
    [_circularScalingSculpt addTarget:self action:@selector(scalingSculptTypeChanged:) forControlEvents:UIControlEventTouchUpInside];
    [_circularScalingSculpt setFrame:CGRectMake(15, nextY + 10, 200, 30)];
    [contentView addSubview:_circularScalingSculpt];
    
    nextY = CGRectGetMaxY(suclptingHeader.frame);
    _silhouetteScalingSculpt = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [_silhouetteScalingSculpt setContentHorizontalAlignment:UIControlContentHorizontalAlignmentLeft];
    [_silhouetteScalingSculpt setTitle:@"Silhouette scaling" forState:UIControlStateNormal];
    [_silhouetteScalingSculpt addTarget:self action:@selector(scalingSculptTypeChanged:) forControlEvents:UIControlEventTouchUpInside];
    [_silhouetteScalingSculpt setFrame:CGRectMake(CGRectGetMaxX(_circularScalingSculpt.frame), nextY + 10, 200, 30)];
    [contentView addSubview:_silhouetteScalingSculpt];
    
    
    
    if ([UIDevice currentDevice].userInterfaceIdiom == UIUserInterfaceIdiomPhone) {
        UIButton* dismissButton = [UIButton buttonWithType:UIButtonTypeRoundedRect];
        [dismissButton setBackgroundColor:[UIColor lightGrayColor]];
        [dismissButton setFrame:CGRectMake(20, self.view.frame.size.height - 60, self.view.frame.size.width - 40, 40)];
        [dismissButton setAutoresizingMask:UIViewAutoresizingFlexibleTopMargin|UIViewAutoresizingFlexibleWidth];
        [dismissButton addTarget:self action:@selector(dismissButtonClicked:) forControlEvents:UIControlEventTouchUpInside];
        [dismissButton setTitle:@"OK" forState:UIControlStateNormal];
        [dismissButton.titleLabel setFont:[UIFont systemFontOfSize:15.0f]];
        [contentView addSubview:dismissButton];
    }
    contentView.contentSize = CGSizeMake(300, CGRectGetMaxY(contentView.frame));
}

-(void)viewWillAppear:(BOOL)animated {
//    [_transformSwitch setOn:PAMSettingsManager::getInstance().transform];
//    [_skeletonSwitch setOn:PAMSettingsManager::getInstance().showSkeleton];
    _smoothingSlider.value = PAMSettingsManager::getInstance().smoothingBrushSize;
    _branchWidth.value = PAMSettingsManager::getInstance().branchWidth;
    _tapSmoothing.value = PAMSettingsManager::getInstance().tapSmoothing;
    _baseSmoothingIterationsSlider.value = PAMSettingsManager::getInstance().baseSmoothingIterations;
    [_spineSmoothing setOn:PAMSettingsManager::getInstance().spineSmoothing];
    [_poleSmoothing setOn:PAMSettingsManager::getInstance().poleSmoothing];
    _silhouetteScalingBrushSize.value = PAMSettingsManager::getInstance().silhouetteScalingBrushSize;
}

-(void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

//-(void)skeletonSwitchClicked:(UIControl*)sender {
//    [self.delegate showSkeleton:_skeletonSwitch.isOn];
//}


-(void)clearModelButton:(UIControl*)sender {
    [self.delegate clearModel];
}

-(void)resetButton:(UIControl*)sender {
    [self.delegate resetTransformations];
}

//-(void)showRibJunctions:(UIControl*)sender {
//    [self.delegate showRibJunctions];
//}

-(void)globalSmoothing:(UIControl*)sender {
    [self.delegate globalSmoothing];
}
-(void)loadArmadillo:(UIControl*)sender {
    [self.delegate loadArmadillo];
}

-(void)subdivide:(UIControl*)sender {
    [self.delegate subdivide];
}

-(void)emailObj:(UIControl*)sender {
    [self.delegate emailObj];
}

#pragma mark - BRANCH CREATION
-(void)spineSmoothing:(UISwitch*)sender {
    [self.delegate spineSmoothing:sender.isOn];
}

-(void)poleSmoothing:(UISwitch*)sender {
    [self.delegate poleSmoothing:sender.isOn];
}

-(void)smoothingBrushSize:(UIControl*)sender {
    UISlider* slider = (UISlider*)sender;
    [self.delegate smoothingBrushSize:slider.value];
}

-(void)baseSmoothingIterations:(UIControl*)sender {
    UISlider* slider = (UISlider*)sender;
    [self.delegate baseSmoothingIterations:slider.value];
}

-(void)branchWidth:(UIControl*)sender {
    UISlider* slider = (UISlider*)sender;
    [self.delegate branchWidth:slider.value];
}

-(void)tapSmoothing:(UIControl*)sender {
    UISlider* slider = (UISlider*)sender;
    [self.delegate tapSmoothing:slider.value];
}
-(void)dismissButtonClicked:(UIControl*)sender {
    [self.delegate dismiss];
}

#pragma mark - SCULPTING
-(void)scalingSculptTypeChanged:(UIControl*)sender {
    if (sender == _silhouetteScalingSculpt) {
        [self.delegate scalingSculptTypeChanged:PAMSettingsManager::ScultpScalingType::Silhouette];
    } else if (sender == _circularScalingSculpt) {
        [self.delegate scalingSculptTypeChanged:PAMSettingsManager::ScultpScalingType::Circular];
    }
}

-(void)silhouetteScalingBrushSize:(UIControl*)sender {
    UISlider* slider = (UISlider*)sender;
    [self.delegate silhouetteScalingBrushSize:slider.value];
}

@end
