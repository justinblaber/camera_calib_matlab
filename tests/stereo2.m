%% Clear
clear, clc;

%% Set images
cb_img_paths.L = {'images/left01.jpg'};
cb_img_paths.R = {'images/right01.jpg'};

% Validate all calibration board images
cb_imgs.L = class.img.validate_similar_imgs(cb_img_paths.L);
cb_imgs.R = class.img.validate_similar_imgs(cb_img_paths.R);
                     
%% Load calibration config file
calib_config = util.read_calib_config('configs/stereo.conf');

%% Get four points in pixel coordinates per calibration board image
four_points_ps.L{1} = [245.4038  95.1070
                       249.9282  254.6160
                       478.6004  87.1966
                       476.3041  265.6548];                            
four_points_ps.R{1} = [128.7735  111.3728
                       136.6242  266.9293
                       345.0131  95.2422
                       346.9742  278.7228];

%% Perform stereo calibration
[calib,R_s,t_s] = alg.stereo_calib_four_points(cb_imgs, ...
                                               four_points_ps, ...
                                               calib_config);
                
%% Save calibration
util.write_stereo_calib_four_points(calib, ...
                                    R_s, ...
                                    t_s, ...
                                    'calibrations/stereo2.txt');
                    
%% Read calibration
clear;

[calib,R_s,t_s] = util.read_stereo_calib_four_points('calibrations/stereo2.txt');

%% Debug with stereo gui
debug.gui_stereo_calib_four_points(calib,R_s,t_s);
