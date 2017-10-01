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
calib_config = util.load_calib_config('configs/stereo.conf');

%% Get four points in pixel coordinates per calibration board image
four_points_ps = {};
switch calib_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(calib_config);

        four_points_ps{1} = [244 94;
                             249 254;
                             479 86;
                             476 264];
                   
        four_points_ps{2} = [251 127;
                             526 182;
                             257 359;
                             439 397];
                   
        four_points_ps{3} = [279 72;
                             189 257;
                             564 154;
                             499 375];
                    
        four_points_ps{4} = [189 131;
                             180 327;
                             471 110;
                             476 338]; 
                   
        four_points_ps{5} = [242 98;
                             437 49;
                             280 378;
                             544 313];
                         
        % Refine
        for i = 1:length(four_points_ps)
            four_points_ps{i} = alg.refine_points(four_points_ps{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_ps{i},calib_config), ...
                                                  calib_config); 
        end   
end

%% Perform single calibration
[A,distortion,rotations,translations,board_points_ps,homographies_refine] = alg.single_calibrate(cb_imgs, ...
                                                                                                 four_points_ps, ...
                                                                                                 calib_config);
                                                                                             
%% Save calibration
util.write_single_calib(cb_imgs, ...
                        board_points_ps, ...
                        four_points_ps, ...
                        A, ...
                        distortion, ...
                        rotations, ...
                        translations, ...
                        homographies_refine, ...
                        calib_config, ...
                        'calibrations/single2.txt');
                    
%% Read calibration
clear;

[cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,homographies_refine,calib_config] = util.read_single_calib('calibrations/single2.txt');

%% Debug with gui
f = figure(1);
debug.gui_single(cb_imgs, ...
                 board_points_ps, ...
                 four_points_ps, ...
                 A, ...
                 distortion, ...
                 rotations, ...
                 translations, ...
                 homographies_refine, ...
                 calib_config, ...
                 f);
