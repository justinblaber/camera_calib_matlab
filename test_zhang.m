%% Clear
clear, clc;
f = figure(1);

%% Set images
cb_img_paths = {'test_images/Image1.tif', ...
                'test_images/Image2.tif', ...
                'test_images/Image3.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
%% Load calibration board config file
cb_config = util.load_cb_config('ncorr_board.yaml');

% Debug
util.debug_cb_config(cb_config,subplot(2,2,1,'parent',f));

%% Get four points in image coordinates per image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        four_points_is{1} = [168 179;
                             135 376;
                             415 194;
                             472 385];
                         
        four_points_is{1} = alg.refine_points(four_points_is{1}, ...
                                              cb_imgs(1), ...
                                              alg.refine_window(four_points_is{1},cb_config));    
        
        four_points_is{2} = [173 106;
                             138 378;
                             443 127;
                             481 389];
                                            
        four_points_is{2} = alg.refine_points(four_points_is{2}, ...
                                              cb_imgs(2), ...
                                              alg.refine_window(four_points_is{2},cb_config)); 
        
        four_points_is{3} = [199 97;
                             117 354;
                             472 142;
                             461 412]; 
                                                            
        four_points_is{3} = alg.refine_points(four_points_is{3}, ...
                                              cb_imgs(3), ...
                                              alg.refine_window(four_points_is{3},cb_config)); 
end

%% Get homographies for four points
% Get four points and board points in world coordinates
[board_points_w, four_points_w] = alg.cb_points(cb_config);

homographies_four_points = {};
for i = 1:length(cb_imgs)
    homographies_four_points{i} = alg.linear_homography(four_points_w,four_points_is{i}); %#ok<SAGROW>
end

%% Refine points
% Apply homography to points
board_points_is = {};
for i = 1:length(cb_imgs)
    board_points_is{i} = alg.apply_homography(homographies_four_points{i},board_points_w); %#ok<SAGROW>
end

% Refine points
for i = 1:length(cb_imgs)
    board_points_is{i} = alg.refine_points(board_points_is{i}, ...
                                           cb_imgs(i), ...
                                           alg.refine_window(four_points_is{i},cb_config));  %#ok<SAGROW>
end

% Debug
for i = 1:length(cb_imgs)
    util.debug_refine_points(board_points_is{i}, ...
                             cb_imgs(i), ...
                             alg.refine_window(four_points_is{i},cb_config), ...
                             subplot(2,2,i+1,'parent',f));
end