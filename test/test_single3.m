%% Set images, load config, and get four corners - single
clear, clc;
f1 = figure(1);

cb_img_paths = {'test/test_images/left01.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration board config file
cb_config = util.load_cb_config('test/board_stereo.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(1,2,1,'parent',f1));

% Get four points in pixel coordinates per calibration board image
four_points_ps = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cb_config);

        four_points_ps{1} = [244 94;
                             249 254;
                             479 86;
                             476 264];
                         
        % Refine
        for i = 1:length(four_points_ps)
            four_points_ps{i} = alg.refine_points(four_points_ps{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_ps{i},cb_config), ...
                                                  cb_config);  %#ok<SAGROW>
          
            debug.plot_cb_refine_points(four_points_ps{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_ps{i},cb_config), ...                                        
                                        cb_config, ...
                                        false, ...
                                        subplot(1,2,i+1,'parent',f1));
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
                              subplot(1,2,i+1,'parent',f2));
end  
% Plot residuals
plot(res(:,1),res(:,2),'bo','parent',subplot(1,2,1,'parent',f2));
set(subplot(1,2,1,'parent',f2),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);
