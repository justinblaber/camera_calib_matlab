%% Set images, load config, and get four corners
clear, clc;
f1 = figure(1);

cb_img_paths = {'test/test_images/left03.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration config file
cal_config = util.load_cal_config('test/stereo.yaml');

% Debug
debug.plot_cb_board_info_2D(cal_config,subplot(1,3,1,'parent',f1));

% Get four points in pixel coordinates per calibration board image
four_points_ps = {};
switch cal_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cal_config);

        % Board 1 
        four_points_ps{1} = [279 70;
                             189 257;
                             564 152;
                             499 374];
                           
        % Refine
        for i = 1:length(four_points_ps)
            four_points_ps{i} = alg.refine_points(four_points_ps{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_ps{i},cal_config), ...
                                                  cal_config);    %#ok<SAGROW>
                                              
            debug.plot_cb_refine_points(four_points_ps{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_ps{i},cal_config), ...
                                        cal_config, ...
                                        true, ...
                                        subplot(1,3,2,'parent',f1));
        end   
end

%% Set images, load config, and get four corners - single
clear, clc;
f1 = figure(1);

cb_img_paths = {'test/test_images/Image5.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration config file
cal_config = util.load_cal_config('test/single.yaml');

% Debug
debug.plot_cb_board_info_2D(cal_config,subplot(1,3,1,'parent',f1));

% Get four points in pixel coordinates per calibration board image
four_points_ps = {};
switch cal_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cal_config);

        % Board 1 
        four_points_ps{1} = [80 194;
                             400 428;
                             222 43;
                             435 218];
                           
        % Refine
        for i = 1:length(four_points_ps)
            four_points_ps{i} = alg.refine_points(four_points_ps{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_ps{i},cal_config), ...
                                                  cal_config);    %#ok<SAGROW>
                                              
            debug.plot_cb_refine_points(four_points_ps{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_ps{i},cal_config), ...
                                        cal_config, ...
                                        true, ...
                                        subplot(1,3,2,'parent',f1));
        end   
end

%% Refine points

% Get homographies for four points ---------------------------------------%
[board_points_w, four_points_w] = alg.cb_points(cal_config);

homographies = {};
for i = 1:length(cb_imgs)
    homographies{i} = alg.homography(four_points_w, ...
                                                 four_points_ps{i}, ...
                                                 cal_config);  %#ok<SAGROW>
end

% Refine points ----------------------------------------------------------%
% Apply homography to points, then refine them
board_points_ps = {};
for i = 1:length(cb_imgs)
    board_points_ps{i} = alg.apply_homography(homographies{i}, ...
                                              board_points_w);  %#ok<SAGROW>
end

% Refine points
for i = 1:length(cb_imgs)    
    board_points_ps{i} = alg.refine_points(board_points_ps{i}, ...
                                           cb_imgs(i), ...
                                           homographies{i}, ...
                                           cal_config);  %#ok<SAGROW>
                                       
    debug.plot_cb_refine_points(board_points_ps{i}, ...
                                cb_imgs(i), ...
                                alg.homography(board_points_w,board_points_ps{i},cal_config), ...
                                cal_config, ...
                                false, ...
                                subplot(1,3,3,'parent',f1));
end
