% This example tests out calibration of a single camera using a single 
% calibration board (not recommended, this is just to test out an edge case)
% by using the "four point method" which is similar to Bouguet's toolbox.

%% Clear
clear, clc;
                     
%% Read calibration config
calib_config = util.read_calib_config('configs/stereo.conf');

%% Set images
cb_img_paths = {'images/stereo/left01.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);

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
util.write_single_calib_four_points(calib,'calibrations/single3.txt');
                    
%% Read calibration
clear;

calib = util.read_single_calib_four_points('calibrations/single3.txt');

%% Debug with gui
debug.gui_single_calib_four_points(calib);
