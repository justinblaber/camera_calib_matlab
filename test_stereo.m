%% Clear
clear, clc;
f1 = figure(1);

%% Set images
cb_img_paths.l = {'test_images/left01.jpg', ...
                  'test_images/left02.jpg', ...
                  'test_images/left03.jpg', ...
                  'test_images/left04.jpg'};
cb_img_paths.r = {'test_images/right01.jpg', ...
                  'test_images/right02.jpg', ...
                  'test_images/right03.jpg', ...
                  'test_images/right04.jpg'};

% Validate all calibration board images
cb_imgs.l = class.img.validate_similar_imgs(cb_img_paths.l);
cb_imgs.r = class.img.validate_similar_imgs(cb_img_paths.r);
                     
%% Load calibration board config file
cb_config = util.load_cb_config('board_stereo.yaml');

% Debug
util.debug_cb_config(cb_config,subplot(5,2,1:2,'parent',f1));

%% Get four points in image coordinates per calibration board image
four_points_is.l = {};
four_points_is.r = {};
switch cb_config.calibration
    case 'four_point_auto'
        error('Automatic four point detection has not been implemented yet');
    case 'four_point_manual'
        % Four points are selected manually; Do refinement of four points 
        % here since automatic detection may not be on corners.
        [~, four_points_w] = alg.cb_points(cb_config);

        % Board 1 
        four_points_is.l{1} = [245 93;
                               250 252;
                               479 85;
                               476 265];                              
        four_points_is.r{1} = [128 110;
                               137 266;
                               346 94;
                               347 278];
       
        % Board 2
        four_points_is.l{2} = [257 358;
                               439 397;
                               252 129;
                               525 182];
        four_points_is.r{2} = [127 367;
                               300 411;
                               71 148;
                               324 191];
                               
        % Board 3
        four_points_is.l{3} = [278 73;
                               187 257;
                               563 154;
                               499 374];
        four_points_is.r{3} = [133 90;
                               42 270;
                               403 161;
                               314 392];
        
        % Board 4
        four_points_is.l{4} = [189 130;
                               181 329;
                               470 112;
                               475 338];
        four_points_is.r{4} = [59 149;
                               48 337;
                               308 120;
                               305 353];
               
        % Refine
        for i = 1:length(cb_imgs.l)
            four_points_is.l{i} = alg.refine_points(four_points_is.l{i}, ...
                                                    cb_imgs.l(i), ...
                                                    alg.linear_homography(four_points_w,four_points_is.l{i}), ...
                                                    cb_config);  
            four_points_is.r{i} = alg.refine_points(four_points_is.r{i}, ...
                                                    cb_imgs.r(i), ...
                                                    alg.linear_homography(four_points_w,four_points_is.r{i}), ...
                                                    cb_config);   
                                                    
            util.debug_cb_refine_points(four_points_is.l{i},cb_imgs.l(i),alg.linear_homography(four_points_w,four_points_is.l{i}),cb_config,subplot(5,2,2*i+1,'parent',f1));
            util.debug_cb_refine_points(four_points_is.r{i},cb_imgs.r(i),alg.linear_homography(four_points_w,four_points_is.r{i}),cb_config,subplot(5,2,2*i+2,'parent',f1));
        end   
end

%% Perform zhang calibration on left and right
[A.l,distortion.l,rotations.l,translations.l,board_points_is.l] = alg.zhang_calibrate(cb_imgs.l,four_points_is.l,cb_config);
[A.r,distortion.r,rotations.r,translations.r,board_points_is.r] = alg.zhang_calibrate(cb_imgs.r,four_points_is.r,cb_config);

% Debug by reprojecting points
f2 = figure(2);
for i = 1:length(cb_imgs.l)
    util.debug_cb_points_disp(alg.apply_full_model(A.l,distortion.l,rotations.l{i},translations.l{i},alg.cb_points(cb_config)), ...
                              board_points_is.l{i}, ...
                              cb_imgs.l(i), ...
                              subplot(4,2,2*i-1,'parent',f2));
    util.debug_cb_points_disp(alg.apply_full_model(A.r,distortion.r,rotations.r{i},translations.r{i},alg.cb_points(cb_config)), ...
                              board_points_is.r{i}, ...
                              cb_imgs.r(i), ...
                              subplot(4,2,2*i,'parent',f2));
end

%% Perform addition stereo refinement

% Get linear guess for R_s
r = zeros(9*length(rotations.l),1);
R = zeros(9*length(rotations.l),9);
for i = 1:length(rotations.l)
    top = (i-1)*9+1;
    R(top:top+2,1:3) = rotations.l{i};
    R(top+3:top+5,4:6) = rotations.l{i};
    R(top+6:top+8,7:9) = rotations.l{i};
    r(top:top+8) = reshape(rotations.r{i},[],1);
end

% Get leasts squares approximation
R_s = reshape(pinv(R)*r,3,3);

% Get best rotation approximation
[U,~,V] = svd(R_s);
R_s = U*V';

% Get linear guess for t
t_s = pinv(vertcat(rotations.l{:})) * (vertcat(translations.r{:})-vertcat(translations.l{:}));

% Nonlinear refinement next
