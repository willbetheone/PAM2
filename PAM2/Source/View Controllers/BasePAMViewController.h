//
//  BasePAMViewController.h
//  PAM2
//
//  Created by Rinat Abdrashitov on 2014-06-11.
//  Copyright (c) 2014 Rinat Abdrashitov. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>
#import "SettingsViewController.h"

@interface BasePAMViewController : GLKViewController <SettingsViewControllerDelegate> {
    GLsizei _glWidth;
    GLsizei _glHeight;
    
    UIViewController* _settingsController;
    UIPopoverController* _settingsPopover;
    
    UILabel* _transformModeLabel;
    UILabel* _hintLabel;
}

@end
