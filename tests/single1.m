%% Clear
clear, clc;

%% Set images
cb_img_paths = {'images/Image1.tif', ...
                'images/Image2.tif', ...
                'images/Image3.tif', ...
                'images/Image4.tif', ...
                'images/Image5.tif', ...
                'images/Image6.tif', ...
                'images/Image7.tif', ...
                'images/Image8.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
%% Load calibration config file
cal_config = util.load_cal_config('configs/single.conf');

%% Get four points in pixel coordinates per calibration board image
four_points_ps = {};
switch cal_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cal_config);

        % Board 1 
        four_points_ps{1} = [168 179;
                             135 376;
                             415 194;
                             472 385];
                                     
        % Board 2 
        four_points_ps{2} = [173 106;
                             138 378;
                             443 127;
                             481 389];
                         
        % Board 3 
        four_points_ps{3} = [199 97;
                             117 354;
                             472 142;
                             461 412]; 
                         
        % Board 4 
        four_points_ps{4} = [250 113;
                             145 359;
                             533 88;
                             509 392]; 
                                
        % Board 5 
        four_points_ps{5} = [84 196;
                             403 429;
                             222 43;
                             432 217]; 
                         
        % Board 6
        four_points_ps{6} = [90 127;
                             117 398;
                             535 172;
                             423 413]; 
                         
        % Board 7
        four_points_ps{7} = [181 128;
                             164 452;
                             482 94;
                             406 350]; 
                         
        % Board 8
        four_points_ps{8} = [72 100;
                             88 426;
                             372 65;
                             325 321]; 
        % Refine
        for i = 1:length(four_points_ps)
            four_points_ps{i} = alg.refine_points(four_points_ps{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_ps{i},cal_config), ...
                                                  cal_config); 
        end   
end

%% Perform single calibration
[A,distortion,rotations,translations,board_points_ps,homographies_refine] = alg.single_calibrate(cb_imgs, ...
                                                                                                 four_points_ps, ...
                                                                                                 cal_config);

%% Debug with gui
f = figure(2);
debug.gui_single(cb_imgs, ...
                 board_points_ps, ...
                 four_points_ps, ...
                 A, ...
                 distortion, ...
                 rotations, ...
                 translations, ...
                 homographies_refine, ...
                 cal_config, ...
                 f);
             
%% Save calibration
util.write_single_calib(cb_imgs, ...
                        board_points_ps, ...
                        four_points_ps, ...
                        A, ...
                        distortion, ...
                        rotations, ...
                        translations, ...
                        homographies_refine, ...
                        cal_config, ...
                        'calibrations/single1.txt');