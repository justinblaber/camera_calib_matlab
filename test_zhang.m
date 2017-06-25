%% Clear
clear, clc;
f = figure(1);

%% Set images
cb_img_paths = {'test_images/Image1.tif', ...
                'test_images/Image2.tif', ...
                'test_images/Image3.tif', ...
                'test_images/Image4.tif', ...
                'test_images/Image5.tif', ...
                'test_images/Image6.tif', ...
                'test_images/Image7.tif', ...
                'test_images/Image8.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
%% Load calibration board config file
cb_config = util.load_cb_config('ncorr_board.yaml');

% Debug
util.debug_cb_config(cb_config,subplot(3,3,1,'parent',f));

%% Get four points in image coordinates per calibration board image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually; Do refinement of four points 
        % here since automatic detection may not be on corners.
        [~, four_points_w] = alg.cb_points(cb_config);

        % Board 1 
        four_points_is{1} = [168 179;
                             135 376;
                             415 194;
                             472 385];
                                     
        % Board 2 
        four_points_is{2} = [173 106;
                             138 378;
                             443 127;
                             481 389];
                         
        % Board 3 
        four_points_is{3} = [199 97;
                             117 354;
                             472 142;
                             461 412]; 
                         
        % Board 4 
        four_points_is{4} = [250 113;
                             145 359;
                             533 88;
                             509 392]; 
                                
        % Board 5 
        four_points_is{5} = [84 196;
                             403 429;
                             222 43;
                             432 217]; 
                         
        % Board 6
        four_points_is{6} = [90 127;
                             117 398;
                             535 172;
                             423 413]; 
                         
        % Board 7
        four_points_is{7} = [181 128;
                             164 452;
                             482 94;
                             406 350]; 
                         
        % Board 8
        four_points_is{8} = [72 100;
                             88 426;
                             372 65;
                             325 321]; 
        % Refine
        for i = length(four_points_is)
            four_points_is{i} = alg.refine_points(four_points_is{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.linear_homography(four_points_w,four_points_is{i}), ...
                                                  cb_config);            
        end   
end

%% Get homographies for four points
[board_points_w, four_points_w] = alg.cb_points(cb_config);

homographies_four_points = {};
for i = 1:length(cb_imgs)
    homographies_four_points{i} = alg.linear_homography(four_points_w,four_points_is{i}); %#ok<SAGROW>
end

%% Refine points
% Apply homography to pocints
board_points_is = {};
for i = 1:length(cb_imgs)
    board_points_is{i} = alg.apply_homography(homographies_four_points{i},board_points_w); %#ok<SAGROW>
end

% Refine points
for i = 1:length(cb_imgs)    
    board_points_is{i} = alg.refine_points(board_points_is{i}, ...
                                           cb_imgs(i), ...
                                           homographies_four_points{i}, ...
                                           cb_config);  %#ok<SAGROW>
end

% Debug
for i = 1:length(cb_imgs)
    util.debug_refine_points(board_points_is{i}, ...
                             cb_imgs(i), ...
                             homographies_four_points{i}, ...
                             cb_config, ...
                             subplot(3,3,i+1,'parent',f));
end

%% Get homographies using refined points
homographies = {};
for i = 1:length(cb_imgs)
    homographies{i} = alg.linear_homography(board_points_w,board_points_is{i}); %#ok<SAGROW>
end

% Debug by reprojecting points and see how close they are to measured points.
for i = 1:length(cb_imgs)
    board_points_i_debug = alg.apply_homography(homographies{i},board_points_w);
    util.debug_cb_points(vertcat(board_points_i_debug,board_points_is{i}),cb_imgs(i));
end

%% Get initial guess for intrinsic camera parameters using all homographies
A = alg.linear_intrinsic_params(homographies);

%% Get rotations and translations per homography
rotations = {};
translations = {};
for i = 1:length(cb_imgs)
    [rotations{i}, translations{i}] = alg.RT_from_homography(homographies{i},A); %#ok<SAGROW>
end

% Debug by recomputing homographies and reprojecting points and see how
% close they are to measured points.
homographies_debug = {};
for i = 1:length(cb_imgs)
    R = rotations{i};
    t = translations{i};
    homography_debug = A*[R(:,1) R(:,2) t];
    homographies_debug{i} = homography_debug; %#ok<SAGROW>
    board_points_i_debug = alg.apply_homography(homography_debug,board_points_w);
    util.debug_cb_points(vertcat(board_points_i_debug,board_points_is{i}),cb_imgs(i));
end
