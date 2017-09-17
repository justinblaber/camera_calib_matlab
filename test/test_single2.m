%% Set images, load config, and get four corners - single
clear, clc;
f1 = figure(1);
clf(f1);

cb_img_paths = {'test/test_images/left01.jpg', ...
                'test/test_images/left02.jpg', ...
                'test/test_images/left03.jpg', ...
                'test/test_images/left04.jpg', ...
                'test/test_images/left05.jpg'};
                     
% Validate all calibration board images
cb_imgs = class.img.validate_similar_imgs(cb_img_paths);
                     
% Load calibration config file
cal_config = util.load_cal_config('test/stereo.yaml');

% Debug
debug.plot_cb_board_info_2D(cal_config,subplot(2,3,1,'parent',f1));

% Get four points in pixel coordinates per calibration board image
four_points_ps = {};
switch cal_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually
        [~, four_points_w] = alg.cb_points(cal_config);

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
                                                  alg.homography(four_points_w,four_points_ps{i},cal_config), ...
                                                  cal_config); 
          
            debug.plot_cb_refine_points(four_points_ps{i}, ...
                                        cb_imgs(i), ...
                                        alg.homography(four_points_w,four_points_ps{i},cal_config), ...                                        
                                        cal_config, ...
                                        false, ...
                                        subplot(2,3,i+1,'parent',f1));
        end   
end

%% Perform single calibration
[A,distortion,rotations,translations,board_points_ps] = alg.single_calibrate(cb_imgs, ...
                                                                             four_points_ps, ...
                                                                             cal_config);

% Debug by reprojecting points
f2 = figure(2);
clf(f2);
res = [];
for i = 1:length(cb_imgs)
    % Get residuals
    p_m = alg.p_m(A, ...
                  distortion, ...
                  rotations{i}, ...
                  translations{i}, ...
                  alg.cb_points(cal_config));
    res = vertcat(res,p_m-board_points_ps{i});   %#ok<AGROW>
    
    % Plot both points
    debug.plot_cb_points_disp(board_points_ps{i}, ...
                              p_m, ...                              
                              cb_imgs(i), ...
                              subplot(2,3,i+1,'parent',f2));
end  
% Plot residuals
plot(res(:,1),res(:,2),'bo','parent',subplot(2,3,1,'parent',f2));
set(subplot(2,3,1,'parent',f2),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect(subplot(2,3,1,'parent',f2),[1 1 1]);


%% Plot extrinsics

f3 = figure(3);
clf(f3);
a = axes(f3);

debug.plot_single_extrinsic(rotations, ...
                            translations, ...
                            0.75*ones(length(cb_imgs),1), ...
                            0.75*ones(length(cb_imgs),1), ...
                            cal_config, ...
                            a);