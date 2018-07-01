%% Clear
clear, clc;

%% Set images
cb_img_paths = {'images/left01.jpg', ...
                'images/left02.jpg', ...
                'images/left03.jpg', ...
                'images/left04.jpg', ...
                'images/left05.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
%% Load calibration config file
calib_config = util.read_calib_config('configs/stereo.conf');

%% Get four points in pixel coordinates per calibration board image
four_points_ps{1} = [245.4038  95.1070
                     249.9282  254.6160
                     478.6004  87.1966
                     476.3041  265.6548];

four_points_ps{2} = [252.0843  128.9690
                     524.5152  182.0328
                     257.1694  358.2938
                     438.8563  397.7544];

four_points_ps{3} = [278.1340  73.1726
                     188.3629  258.4665
                     563.2866  154.6281
                     498.8266  375.6217];

four_points_ps{4} = [189.5038  131.6406
                     180.3775  329.1699
                     471.2737  111.2002
                     476.1010  339.2265]; 

four_points_ps{5} = [241.9079  98.0027
                     437.3147  50.8201
                     280.5987  379.7500
                     543.3684  314.8763];

%% Perform single calibration
calib = alg.single_calib_four_points(cb_imgs, ...
                                     four_points_ps, ...
                                     calib_config);
                                                                                             
%% Save calibration
util.write_single_calib_four_points(calib,'calibrations/single2.txt');
                    
%% Read calibration
clear;

calib = util.read_single_calib_four_points('calibrations/single2.txt');

%% Debug with gui
debug.gui_single_calib_four_points(calib);
