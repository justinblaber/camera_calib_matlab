%% Clear
clear, clc, close all;

%% Set images
cb_img_paths = {'test_images/Image1.tif', ...
                'test_images/Image2.tif', ...
                'test_images/Image3.tif', ...
                'test_images/Image4.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
%%  Load calibration board config file
cb_config = util.load_cb_config('ncorr_board.yaml');

% Debug
util.debug_cb_config(cb_config);

%%  Get four points in image coordinates per image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        four_points_is{1} = [148 167;
                             97 404;
                             430 186;
                             508 409];
                                            
        four_points_is{2} = [150 89;
                             101 405;
                             462 115;
                             511 418];
                                            
        four_points_is{3} = [181 79;
                             79 374;
                             496 130;
                             491 447];   
                      
        four_points_is{4} = [234 101;
                             108 383;
                             562 69;
                             545 433];
end

%%  Get homographies for four points
% Get four points and board points in world coordinates
[board_points_w, four_points_w] = util.cb_points(cb_config);

homographies_four_points = {};
for i = 1:length(cb_imgs)
    homographies_four_points{i} = alg.linear_homography(four_points_w,four_points_is{i}); %#ok<SAGROW>
end

for i = 1:length(cb_imgs)
    util.debug_cb_points(alg.apply_homography(homographies_four_points{i},vertcat(four_points_w,board_points_w)),cb_imgs(i));
end