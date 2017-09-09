%% Set images, load config, and get four corners - single
clear, clc;
f1 = figure(1);

cb_img_paths = {'test/test_images/Image1.tif', ...
                'test/test_images/Image2.tif', ...
                'test/test_images/Image3.tif', ...
                'test/test_images/Image4.tif', ...
                'test/test_images/Image5.tif', ...
                'test/test_images/Image6.tif', ...
                'test/test_images/Image7.tif', ...
                'test/test_images/Image8.tif'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration board config file
cb_config = util.load_cb_config('test/board_single.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(3,3,1,'parent',f1));

% Get four points in pixel coordinates per calibration board image
four_points_ps = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cb_config);

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
                                                  alg.homography(four_points_w,four_points_ps{i},cb_config), ...
                                                  cb_config); 
          
            debug.plot_cb_refine_points(four_points_ps{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_ps{i},cb_config), ...                                        
                                        cb_config, ...
                                        false, ...
                                        subplot(3,3,i+1,'parent',f1));
        end   
end

%% Perform single calibration
[A,distortion,rotations,translations,board_points_ps] = alg.single_calibrate(cb_imgs, ...
                                                                             four_points_ps, ...
                                                                             cb_config);

% Debug by reprojecting points
f2 = figure(2);
res = [];
for i = 1:length(cb_imgs)
    % Get residuals
    p_m = alg.p_m(A, ...
                  distortion, ...
                  rotations{i}, ...
                  translations{i}, ...
                  alg.cb_points(cb_config));
    res = vertcat(res,p_m-board_points_ps{i});   %#ok<AGROW>
    
    % Plot both points
    debug.plot_cb_points_disp(board_points_ps{i}, ...
                              p_m, ...                              
                              cb_imgs(i), ...
                              subplot(3,3,i+1,'parent',f2));
end  
% Plot residuals
plot(res(:,1),res(:,2),'bo','parent',subplot(3,3,1,'parent',f2));
set(subplot(3,3,1,'parent',f2),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);
