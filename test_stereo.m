%% Clear
clear, clc;
f1 = figure(1);

%% Set images
cb_img_paths.L = {'test_images/left01.jpg', ...
                  'test_images/left02.jpg', ...
                  'test_images/left03.jpg', ...
                  'test_images/left04.jpg', ...
                  'test_images/left05.jpg'};
cb_img_paths.R = {'test_images/right01.jpg', ...
                  'test_images/right02.jpg', ...
                  'test_images/right03.jpg', ...
                  'test_images/right04.jpg', ...
                  'test_images/right05.jpg'};

% Validate all calibration board images
cb_imgs.L = class.img.validate_similar_imgs(cb_img_paths.L);
cb_imgs.R = class.img.validate_similar_imgs(cb_img_paths.R);
                     
%% Load calibration board config file
cb_config = util.load_cb_config('board_stereo.yaml');

% Debug
debug.plot_cb_config(cb_config,subplot(6,2,1:2,'parent',f1));

%% Get four points in image coordinates per calibration board image
four_points_is.L = {};
four_points_is.R = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually; Do refinement of four points 
        % here since automatic detection may not be on corners.
        [~, four_points_w] = alg.cb_points(cb_config);

        four_points_is.L{1} = [244 94;
                               249 254;
                               479 86;
                               476 264];                            
        four_points_is.R{1} = [128 110;
                               137 266;
                               346 94;
                               347 278];
       
        four_points_is.L{2} = [251 127;
                               526 182;
                               257 359;
                               439 397];
        four_points_is.R{2} = [71 148;
                               324 191;
                               128 367;
                               301 411];
        
        four_points_is.L{3} = [279 72;
                               189 257;
                               564 154;
                               499 375];
        four_points_is.R{3} = [134 89;
                               42 270;
                               403 160;
                               314 392];
                           
        four_points_is.L{4} = [189 131;
                               180 327;
                               471 110;
                               476 338]; 
        four_points_is.R{4} = [59 149;
                               48 337;
                               309 120;
                               306 353];
               
        four_points_is.L{5} = [242 98;
                               437 49;
                               280 378;
                               544 313]; 
        four_points_is.R{5} = [103 114;
                               289 58;
                               99 384;
                               351 330];
                           
        % Refine
        for i = 1:length(cb_imgs.L)
            four_points_is.L{i} = alg.refine_points(four_points_is.L{i}, ...
                                                    cb_imgs.L(i), ...
                                                    alg.homography(four_points_w,four_points_is.L{i},cb_config), ...
                                                    cb_config);  
            four_points_is.R{i} = alg.refine_points(four_points_is.R{i}, ...
                                                    cb_imgs.R(i), ...
                                                    alg.homography(four_points_w,four_points_is.R{i},cb_config), ...
                                                    cb_config);   
                                                    
            debug.plot_cb_refine_points(four_points_is.L{i}, ...
                                        cb_imgs.L(i), ...
                                        alg.homography(four_points_w,four_points_is.L{i},cb_config), ...
                                        cb_config, ...
                                        false, ...
                                        subplot(6,2,2*i+1,'parent',f1));
            debug.plot_cb_refine_points(four_points_is.R{i}, ...
                                        cb_imgs.R(i), ...
                                        alg.homography(four_points_w,four_points_is.R{i},cb_config), ...
                                        cb_config, ...
                                        false, ...
                                        subplot(6,2,2*i+2,'parent',f1));
        end   
end

%% Perform zhang calibration on left and right
[A.L,distortion.L,rotations.L,translations.L,board_points_is.L] = alg.single_calibrate(cb_imgs.L, ...
                                                                                       four_points_is.L, ...
                                                                                       cb_config);
                                                                                  
[A.R,distortion.R,rotations.R,translations.R,board_points_is.R] = alg.single_calibrate(cb_imgs.R, ...
                                                                                       four_points_is.R, ...
                                                                                       cb_config);
% Debug by reprojecting points
f2 = figure(2);
res.L = [];
res.R = [];
for i = 1:length(cb_imgs.L)
    % Get residuals
    p_m.L = alg.p_m(A.L, ...
                    distortion.L, ...
                    rotations.L{i}, ...
                    translations.L{i}, ...
                    alg.cb_points(cb_config));
    res.L = vertcat(res.L,p_m.L-board_points_is.L{i});   
    
    p_m.R = alg.p_m(A.R, ...
                    distortion.R, ...
                    rotations.R{i}, ...
                    translations.R{i}, ...
                    alg.cb_points(cb_config));
    res.R = vertcat(res.R,p_m.R-board_points_is.R{i});   
    
    % Plot both points
    debug.plot_cb_points_disp(board_points_is.L{i}, ...
                              p_m.L, ...                              
                              cb_imgs.L(i), ...
                              subplot(6,2,2*i+1,'parent',f2));
                          
    debug.plot_cb_points_disp(board_points_is.R{i}, ...
                              p_m.R, ...                              
                              cb_imgs.R(i), ...
                              subplot(6,2,2*i+2,'parent',f2));
end  
% Plot residuals
plot(res.L(:,1),res.L(:,2),'bo','parent',subplot(6,2,1,'parent',f2));
set(subplot(6,2,1,'parent',f2),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);

plot(res.R(:,1),res.R(:,2),'bo','parent',subplot(6,2,2,'parent',f2));
set(subplot(6,2,2,'parent',f2),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);

%% Perform stereo refinement

% Get least squares linear guess for R_s
r = [];
R = [];
for i = 1:length(rotations.L)
    r = vertcat(r,rotations.R{i}(:)); %#ok<AGROW>
    R = vertcat(R,[rotations.L{i}(1,1)*eye(3) rotations.L{i}(2,1)*eye(3) rotations.L{i}(3,1)*eye(3);
                   rotations.L{i}(1,2)*eye(3) rotations.L{i}(2,2)*eye(3) rotations.L{i}(3,2)*eye(3);
                   rotations.L{i}(1,3)*eye(3) rotations.L{i}(2,3)*eye(3) rotations.L{i}(3,3)*eye(3)]); %#ok<AGROW>
end

% Get least squares approximation
R_s = reshape(pinv(R)*r,3,3);

% R_s is not necessarily orthogonal, so get the best rotational 
% approximation.
R_s = alg.approx_rot(R_s);

% Get least squares linear guess for t
t = [];
T = [];
for i = 1:length(rotations.L)
    t = vertcat(t,translations.R{i}-rotations.R{i}*rotations.L{i}'*translations.L{i}); %#ok<AGROW>
    T = vertcat(T,eye(3)); %#ok<AGROW>
end

% Get leasts squares approximation
t_s = pinv(T)*t;

% Debug initial guesses
f3 = figure(3);
res.L = [];
res.R = [];
for i = 1:length(cb_imgs.L)
    % Get residuals
    p_m.L = alg.p_m(A.L, ...
                    distortion.L, ...
                    rotations.L{i}, ...
                    translations.L{i}, ...
                    alg.cb_points(cb_config));
    res.L = vertcat(res.L,p_m.L-board_points_is.L{i});   
    
    p_m.R = alg.p_m(A.R, ...
                    distortion.R, ...
                    R_s*rotations.L{i}, ...
                    R_s*translations.L{i}+t_s, ...
                    alg.cb_points(cb_config));
    res.R = vertcat(res.R,p_m.R-board_points_is.R{i});   
    
    % Plot both points
    debug.plot_cb_points_disp(board_points_is.L{i}, ...
                              p_m.L, ...                              
                              cb_imgs.L(i), ...
                              subplot(6,2,2*i+1,'parent',f3));
                          
    debug.plot_cb_points_disp(board_points_is.R{i}, ...
                              p_m.R, ...                              
                              cb_imgs.R(i), ...
                              subplot(6,2,2*i+2,'parent',f3));
end  
% Plot residuals
plot(res.L(:,1),res.L(:,2),'bo','parent',subplot(6,2,1,'parent',f3));
set(subplot(6,2,1,'parent',f3),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);

plot(res.R(:,1),res.R(:,2),'bo','parent',subplot(6,2,2,'parent',f3));
set(subplot(6,2,2,'parent',f3),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);

% Perform nonlinear refinement of all parameters ---------------------%
% Initialize distortion parameters to zero

% Perform full optimization
disp('--------------------------------------------');
[A,distortion,rotations,translations,R_s,t_s] = alg.refine_stereo_params(A, ...
                                                                         distortion, ...
                                                                         rotations, ...
                                                                         translations, ...
                                                                         R_s, ...
                                                                         t_s, ....
                                                                         board_points_is, ...
                                                                         'full', ...
                                                                         cb_config);

% Debug refined parameters
f4 = figure(4);
res.L = [];
res.R = [];
for i = 1:length(cb_imgs.L)
    % Get residuals
    p_m.L = alg.p_m(A.L, ...
                    distortion.L, ...
                    rotations.L{i}, ...
                    translations.L{i}, ...
                    alg.cb_points(cb_config));
    res.L = vertcat(res.L,p_m.L-board_points_is.L{i});   
    
    p_m.R = alg.p_m(A.R, ...
                    distortion.R, ...
                    R_s*rotations.L{i}, ...
                    R_s*translations.L{i}+t_s, ...
                    alg.cb_points(cb_config));
    res.R = vertcat(res.R,p_m.R-board_points_is.R{i});   
    
    % Plot both points
    debug.plot_cb_points_disp(board_points_is.L{i}, ...
                              p_m.L, ...                              
                              cb_imgs.L(i), ...
                              subplot(6,2,2*i+1,'parent',f4));
                          
    debug.plot_cb_points_disp(board_points_is.R{i}, ...
                              p_m.R, ...                              
                              cb_imgs.R(i), ...
                              subplot(6,2,2*i+2,'parent',f4));
end  
% Plot residuals
plot(res.L(:,1),res.L(:,2),'bo','parent',subplot(6,2,1,'parent',f4));
set(subplot(6,2,1,'parent',f4),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);

plot(res.R(:,1),res.R(:,2),'bo','parent',subplot(6,2,2,'parent',f4));
set(subplot(6,2,2,'parent',f4),'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
daspect([1 1 1]);