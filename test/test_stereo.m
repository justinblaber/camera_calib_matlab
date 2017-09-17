%% Clear
clear, clc;
f1 = figure(1);
clf(f1);

%% Set images
cb_img_paths.L = {'test/test_images/left01.jpg', ...
                  'test/test_images/left02.jpg', ...
                  'test/test_images/left03.jpg', ...
                  'test/test_images/left04.jpg', ...
                  'test/test_images/left05.jpg'};
cb_img_paths.R = {'test/test_images/right01.jpg', ...
                  'test/test_images/right02.jpg', ...
                  'test/test_images/right03.jpg', ...
                  'test/test_images/right04.jpg', ...
                  'test/test_images/right05.jpg'};

% Validate all calibration board images
cb_imgs.L = class.img.validate_similar_imgs(cb_img_paths.L);
cb_imgs.R = class.img.validate_similar_imgs(cb_img_paths.R);
                     
%% Load calibration config file
cal_config = util.load_cal_config('stereo.yaml');

% Debug
debug.plot_cb_board_info_2D(cal_config,subplot(6,2,1:2,'parent',f1));

%% Get four points in pixel coordinates per calibration board image
four_points_ps.L = {};
four_points_ps.R = {};
switch cal_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually; Do refinement of four points 
        % here since automatic detection may not be on corners.
        [~, four_points_w] = alg.cb_points(cal_config);

        four_points_ps.L{1} = [244 94;
                               249 254;
                               479 86;
                               476 264];                            
        four_points_ps.R{1} = [128 110;
                               137 266;
                               346 94;
                               347 278];
       
        four_points_ps.L{2} = [251 127;
                               526 182;
                               257 359;
                               439 397];
        four_points_ps.R{2} = [71 148;
                               324 191;
                               128 367;
                               301 411];
        
        four_points_ps.L{3} = [279 72;
                               189 257;
                               564 154;
                               499 375];
        four_points_ps.R{3} = [134 89;
                               42 270;
                               403 160;
                               314 392];
                           
        four_points_ps.L{4} = [189 131;
                               180 327;
                               471 110;
                               476 338]; 
        four_points_ps.R{4} = [59 149;
                               48 337;
                               309 120;
                               306 353];
               
        four_points_ps.L{5} = [242 98;
                               437 49;
                               280 378;
                               544 313]; 
        four_points_ps.R{5} = [103 114;
                               289 58;
                               99 384;
                               351 330];
                           
        % Refine
        for i = 1:length(cb_imgs.L)
            four_points_ps.L{i} = alg.refine_points(four_points_ps.L{i}, ...
                                                    cb_imgs.L(i), ...
                                                    alg.homography(four_points_w,four_points_ps.L{i},cal_config), ...
                                                    cal_config);  
            four_points_ps.R{i} = alg.refine_points(four_points_ps.R{i}, ...
                                                    cb_imgs.R(i), ...
                                                    alg.homography(four_points_w,four_points_ps.R{i},cal_config), ...
                                                    cal_config);   
                                                    
            debug.plot_cb_refine_points(four_points_ps.L{i}, ...
                                        cb_imgs.L(i), ...
                                        alg.homography(four_points_w,four_points_ps.L{i},cal_config), ...
                                        cal_config, ...
                                        false, ...
                                        subplot(6,2,2*i+1,'parent',f1));
            debug.plot_cb_refine_points(four_points_ps.R{i}, ...
                                        cb_imgs.R(i), ...
                                        alg.homography(four_points_w,four_points_ps.R{i},cal_config), ...
                                        cal_config, ...
                                        false, ...
                                        subplot(6,2,2*i+2,'parent',f1));
        end   
end

%% Perform stereo calibration
[A,distortion,rotations,translations,board_points_ps,R_s,t_s,res] = alg.stereo_calibrate(cb_imgs, ...
                                                                                         four_points_ps, ...
                                                                                         cal_config);
                
% Set colors for calibration boards                                                                         
colors = util.distinguishable_colors(length(rotations.L),'w');

% Debug refined parameters
f2 = figure(2);
clf(f2);
for i = 1:length(cb_imgs.L)    
    % Left
    p_m_L = alg.p_m(A.L, ...
                    distortion.L, ...
                    rotations.L{i}, ...
                    translations.L{i}, ...
                    alg.cb_points(cal_config));
    debug.plot_cb_points_disp(board_points_ps.L{i}, ...
                              p_m_L, ...                              
                              cb_imgs.L(i), ...
                              subplot(6,2,2*i+1,'parent',f2));
                          
  	% Right    
    p_m_R = alg.p_m(A.R, ...
                    distortion.R, ...
                    R_s*rotations.L{i}, ...
                    R_s*translations.L{i}+t_s, ...
                    alg.cb_points(cal_config));
    debug.plot_cb_points_disp(board_points_ps.R{i}, ...
                              p_m_R, ...                              
                              cb_imgs.R(i), ...
                              subplot(6,2,2*i+2,'parent',f2));
end  
% Plot residuals
debug.plot_res(res.L,colors,[0.25 0.25 0.25 1 0.25],subplot(6,2,1,'parent',f2));
debug.plot_res(res.R,colors,[0.25 0.25 0.25 1 0.25],subplot(6,2,2,'parent',f2));

%% Plot extrinsics

f3 = figure(3);
clf(f3);
a = axes(f3);

% Reload cal_config incase changes are made
debug.plot_stereo_extrinsic(rotations, ...
                            translations, ...
                            R_s, ...
                            t_s, ...
                            colors, ...
                            [0.25 0.25 0.25 1 0.25], ...
                            cal_config, ...
                            a);
