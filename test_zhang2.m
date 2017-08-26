%% Set images, load config, and get four corners - zhang
clear, clc;
f1 = figure(1);

cb_img_paths = {'test_images/left01.jpg', ...
                'test_images/left02.jpg', ...
                'test_images/left03.jpg', ...
                'test_images/left04.jpg', ...
                'test_images/left05.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration board config file
cb_config = util.load_cb_config('board_stereo.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(2,3,1,'parent',f1));

% Get four points in image coordinates per calibration board image
four_points_is = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cb_config);

        four_points_is{1} = [244 94;
                             249 254;
                             479 86;
                             476 264];
                   
        four_points_is{2} = [251 127;
                             526 182;
                             257 359;
                             439 397];
                   
        four_points_is{3} = [279 72;
                             189 257;
                             564 154;
                             499 375];
                    
        four_points_is{4} = [189 131;
                             180 327;
                             471 110;
                             476 338]; 
                   
        four_points_is{5} = [242 98;
                             437 49;
                             280 378;
                             544 313];
                         
        % Refine
        for i = 1:length(four_points_is)
            four_points_is{i} = alg.refine_points(four_points_is{i}, ...
                                                  cb_imgs(i), ...
                                                  alg.homography(four_points_w,four_points_is{i},cb_config), ...
                                                  cb_config); 
          
            debug.plot_cb_refine_points(four_points_is{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_is{i},cb_config), ...                                        
                                        cb_config, ...
                                        false, ...
                                        subplot(2,3,i+1,'parent',f1));
        end   
end

%% Perform zhang calibration
[rotations,translations,A,distortion,board_points_is] = alg.single_calibrate(cb_imgs, ...
                                                                             four_points_is, ...
                                                                             cb_config);

% Debug by reprojecting points
f2 = figure(2);
res = [];
for i = 1:length(cb_imgs)
    % Get residuals
    model_points_i = alg.apply_full_model(A,distortion,rotations{i},translations{i},alg.cb_points(cb_config));
    res = vertcat(res,model_points_i-board_points_is{i});   %#ok<AGROW>
    
    % Plot both points
    debug.plot_cb_points_disp(board_points_is{i}, ...
                              model_points_i, ...                              
                              cb_imgs(i), ...
                              subplot(2,3,i+1,'parent',f2));
end  
% Plot residuals
plot(res(:,1),res(:,2),'bo','parent',subplot(2,3,1,'parent',f2));
set(subplot(2,3,1,'parent',f2),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);