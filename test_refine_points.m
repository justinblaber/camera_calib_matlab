%% Set images, load config, and get four corners - stereo
clear, clc;
f1 = figure(1);

cb_img_paths = {'test_images/left03.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration board config file
cb_config = util.load_cb_config('board_stereo.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(1,3,1,'parent',f1));

% Get four points in image coordinates per calibration board image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cb_config);

        % Board 1 
        four_points_is{1} = [320 165;
                             266 290;
                             540 232;
                             499 374];
                           
        % Refine
        for i = 1:length(four_points_is)
            four_points_is{i} = alg.refine_points(four_points_is{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_is{i},cb_config), ...
                                                  cb_config);    %#ok<SAGROW>
                                              
            debug.plot_cb_refine_points(four_points_is{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_is{i},cb_config), ...
                                        cb_config, ...
                                        true, ...
                                        subplot(1,3,2,'parent',f1));
        end   
end

%% Set images, load config, and get four corners - zhang
clear, clc;
f1 = figure(1);

cb_img_paths = {'test_images/Image1.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration board config file
cb_config = util.load_cb_config('board_zhang.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(1,2,1,'parent',f1));

% Get four points in image coordinates per calibration board image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cb_config);

        % Board 1 
        four_points_is{1} = [168 178;
                             135 375;
                             414 193;
                             473 385];
                           
        % Refine
        for i = 1:length(four_points_is)
            four_points_is{i} = alg.refine_points(four_points_is{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_is{i},cb_config), ...
                                                  cb_config);    %#ok<SAGROW>
                                              
            debug.plot_cb_refine_points(four_points_is{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_is{i},cb_config), ...
                                        cb_config, ...
                                        true, ...
                                        subplot(1,3,2,'parent',f1));
        end   
end

%% Refine points

% Get homographies for four points ---------------------------------------%
[board_points_w, four_points_w] = alg.cb_points(cb_config);

homographies_four_points = {};
for i = 1:length(cb_imgs)
    homographies_four_points{i} = alg.homography(four_points_w, ...
                                                 four_points_is{i}, ...
                                                 cb_config);  %#ok<SAGROW>
end

% Refine points ----------------------------------------------------------%
% Apply homography to points, then refine them
board_points_is = {};
for i = 1:length(cb_imgs)
    board_points_is{i} = alg.apply_homography(homographies_four_points{i}, ...
                                              board_points_w);  %#ok<SAGROW>
end

% Refine points
for i = 1:length(cb_imgs)    
    board_points_is{i} = alg.refine_points(board_points_is{i}, ...
                                           cb_imgs(i), ...
                                           homographies_four_points{i}, ...
                                           cb_config);  %#ok<SAGROW>
end

debug.plot_cb_refine_points(board_points_is{i}, ...
                            cb_imgs(i), ...
                            alg.homography(board_points_w,board_points_is{i},cb_config), ...
                            cb_config, ...
                            false, ...
                            subplot(1,3,3,'parent',f1));