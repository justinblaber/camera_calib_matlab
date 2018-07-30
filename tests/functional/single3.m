% This example tests out calibration of a single camera using a single 
% calibration board (not recommended; this is just to test out an edge 
% case) by using the "four point method" which is similar to Bouguet's 
% toolbox.

%% Clear
clear, clc;
                     
%% Get test path and name
[test_path, test_name] = fileparts(mfilename('fullpath'));

%% Read calibration config
calib_config = util.read_calib_config(fullfile(test_path,'configs','stereo.conf'));

%% Set images
img_dir_path = fullfile(test_path,'images','stereo');
num_imgs = 1;
for i = 1:num_imgs
    cb_img_paths{i} = fullfile(img_dir_path,['left0' num2str(i) '.jpg']); %#ok<SAGROW>
end     
                     
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
calib_path = fullfile(test_path,'calibrations',[test_name '.txt']);
util.write_single_calib_four_points(calib,calib_path);
                    
%% Read calibration
clearvars -except calib_path;

calib = util.read_single_calib_four_points(calib_path);

%% Debug with gui
debug.gui_single_calib_four_points(calib);
