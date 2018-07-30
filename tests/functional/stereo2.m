% This example tests out calibration of a stereo camera using a single 
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
    cb_img_paths.L{i} = fullfile(img_dir_path,['left0' num2str(i) '.jpg']);
    cb_img_paths.R{i} = fullfile(img_dir_path,['right0' num2str(i) '.jpg']);
end     

% Validate all calibration board images
cb_imgs.L = class.img.validate_similar_imgs(cb_img_paths.L);
cb_imgs.R = class.img.validate_similar_imgs(cb_img_paths.R);

%% Get four points in pixel coordinates per calibration board image
four_points_ps.L{1} = [245.4038  95.1070
                       249.9282  254.6160
                       478.6004  87.1966
                       476.3041  265.6548];                            
four_points_ps.R{1} = [128.7735  111.3728
                       136.6242  266.9293
                       345.0131  95.2422
                       346.9742  278.7228];

four_points_ps.L{2} = [252.0843  128.9690
                       524.5152  182.0328
                       257.1694  358.2938
                       438.8563  397.7544];
four_points_ps.R{2} = [71.4858   148.5386
                       323.8698  192.3194
                       128.0974  367.5743
                       300.7801  412.2106];

four_points_ps.L{3} = [278.1340  73.1726
                       188.3629  258.4665
                       563.2866  154.6281
                       498.8266  375.6217];
four_points_ps.R{3} = [134.1454  90.5307
                       42.7467   270.7947
                       403.1969  161.8248
                       314.5314  392.5653];

four_points_ps.L{4} = [189.5038  131.6406
                       180.3775  329.1699
                       471.2737  111.2002
                       476.1010  339.2265]; 
four_points_ps.R{4} = [59.3682   149.9321
                       47.9803   337.7430
                       308.5367  120.6371
                       305.7566  354.4247];

four_points_ps.L{5} = [241.9079  98.0027
                       437.3147  50.8201
                       280.5987  379.7500
                       543.3684  314.8763]; 
four_points_ps.R{5} = [102.6426  116.3835
                       289.0113  60.3427
                       98.5459   386.5158
                       352.2258  331.3822];

%% Perform stereo calibration
[calib,R_s,t_s] = alg.stereo_calib_four_points(cb_imgs, ...
                                               four_points_ps, ...
                                               calib_config);
                
%% Save calibration
calib_path = fullfile(test_path,'calibrations',[test_name '.txt']);
util.write_stereo_calib_four_points(calib, ...
                                    R_s, ...
                                    t_s, ...
                                    calib_path);
                    
%% Read calibration
clearvars -except calib_path;

[calib,R_s,t_s] = util.read_stereo_calib_four_points(calib_path);

%% Debug with stereo gui
debug.gui_stereo_calib_four_points(calib,R_s,t_s);
