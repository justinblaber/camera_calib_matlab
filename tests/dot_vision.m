% This example tests out calibration of a stereo camera. It also 
% demonstrates how to use the automatic four point detection method and 
% also demonstrates how precalibrations can be done on each camera 
% individually and then the respective intrinsics can be passed into the 
% stereo calibration so they are not re-optimized. This is useful because 
% cameras can be pre-calibrated using lots of images, and then stereo 
% calibration can be done later if the user wants to change the baseline 
% or camera angles.

%% Clear
clear, clc;

%% Set environment
addpath('~/camera_calib/')

%% Read calibration config
calib_config = util.read_calib_config('configs/dot_vision.conf');

%% Calibration of left camera

cb_img_paths_L = { ...
'images/dot_vision/16276941_2018-07-12_01:34:06_883766_1_L.png', ...
'images/dot_vision/16276941_2018-07-12_01:34:19_464618_2_L.png' ...
};

% Validate all calibration board images
cb_imgs_L = class.img.validate_similar_imgs(cb_img_paths_L);
                    
% Get four points
[four_points_ps_L,four_points_debugs_L] = alg.single_four_points_detect(cb_imgs_L, ...
                                                                        calib_config);

% Debug four points
debug.gui_single_four_points_detect(four_points_ps_L, ...
                                    four_points_debugs_L, ...
                                    cb_imgs_L, ...
                                    calib_config);


% Perform calibration
calib_L = alg.single_calib_four_points(cb_imgs_L, ...
                                       four_points_ps_L, ...
                                       calib_config);

% Debug
debug.gui_single_calib_four_points(calib_L);
                                   
%% Calibration of right camera

cb_img_paths_R = {
'images/dot_vision/16276942_2018-07-12_01:34:06_885104_1_R.png', ...
'images/dot_vision/16276942_2018-07-12_01:34:19_465629_2_R.png' ...
};

% Validate all calibration board images
cb_imgs_R = class.img.validate_similar_imgs(cb_img_paths_R);
                    
% Get four points
[four_points_ps_R,four_points_debugs_R] = alg.single_four_points_detect(cb_imgs_R, ...
                                                                        calib_config);

% Debug four points
debug.gui_single_four_points_detect(four_points_ps_R, ...
                                    four_points_debugs_R, ...
                                    cb_imgs_R, ...
                                    calib_config);

% Perform calibration
calib_R = alg.single_calib_four_points(cb_imgs_R, ...
                                       four_points_ps_R, ...
                                       calib_config);
                                                                     
% Debug
debug.gui_single_calib_four_points(calib_R);

%% Stereo calibration

cb_img_paths.L = {
'images/dot_vision/16276941_2018-07-12_01:34:37_254931_3_L.png', ...
'images/dot_vision/16276941_2018-07-12_01:34:54_380623_4_L.png' ...
};
cb_img_paths.R = {
'images/dot_vision/16276942_2018-07-12_01:34:37_255926_3_R.png', ...
'images/dot_vision/16276942_2018-07-12_01:34:54_382094_4_R.png' ...
};

% Validate all calibration board images
cb_imgs.L = class.img.validate_similar_imgs(cb_img_paths.L);
cb_imgs.R = class.img.validate_similar_imgs(cb_img_paths.R);
                 
% Get four points
[four_points_ps,four_points_debugs] = alg.stereo_four_points_detect(cb_imgs, ...
                                                                    calib_config);

% Debug four points
debug.gui_stereo_four_points_detect(four_points_ps, ...
                                    four_points_debugs, ...
                                    cb_imgs, ...
                                    calib_config);
                                
% Perform calibration of extrinsics only
intrin.L = calib_L.intrin;
intrin.R = calib_R.intrin;
[calib,R_s,t_s] = alg.stereo_calib_four_points(cb_imgs, ...
                                               four_points_ps, ...
                                               calib_config, ...
                                               intrin);

% Debug
debug.gui_stereo_calib_four_points(calib,R_s,t_s);

% Perform full calibration
[calib,R_s,t_s] = alg.stereo_calib_four_points(cb_imgs, ...
                                               four_points_ps, ...
                                               calib_config);

% Debug
debug.gui_stereo_calib_four_points(calib,R_s,t_s);