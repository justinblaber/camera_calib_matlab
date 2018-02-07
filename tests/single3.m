%% Clear
clear, clc;

%% Set images
cb_img_paths = {'images/left01.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
%% Load calibration config file
calib_config = util.read_calib_config('configs/stereo.conf');

%% Get four points in pixel coordinates per calibration board image
four_points_ps{1} = [245.4038  95.1070
                     249.9282  254.6160
                     478.6004  87.1966
                     476.3041  265.6548];

%% Perform single calibration
calib = alg.single_calib_four_points(cb_imgs, ...
                                     four_points_ps, ...
                                     calib_config);
                                                                                             
%% Save calibration
util.write_single_calib(calib,'calibrations/single3.txt');
                    
%% Read calibration
clear;

calib = util.read_single_calib('calibrations/single3.txt');

%% Debug with gui
debug.gui_single_calib(calib);
