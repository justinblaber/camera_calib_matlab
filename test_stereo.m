%% Clear
clear, clc;
f1 = figure(1);

%% Set images
cb_img_paths.L = {'test_images/left01.jpg', ...
                  'test_images/left03.jpg', ...
                  'test_images/left04.jpg'};
cb_img_paths.R = {'test_images/right01.jpg', ...
                  'test_images/right03.jpg', ...
                  'test_images/right04.jpg'};

% Validate all calibration board images
cb_imgs.L = class.img.validate_similar_imgs(cb_img_paths.L);
cb_imgs.R = class.img.validate_similar_imgs(cb_img_paths.R);
                     
%% Load calibration board config file
cb_config = util.load_cb_config('board_stereo.yaml');

% Debug
util.debug_cb_config(cb_config,subplot(4,2,1:2,'parent',f1));

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

        % Board 1 
        four_points_is.L{1} = [245 93;
                               250 252;
                               479 85;
                               476 265];                              
        four_points_is.R{1} = [128 110;
                               137 266;
                               346 94;
                               347 278];
       
        % Board 2
        four_points_is.L{2} = [278 73;
                               187 257;
                               563 154;
                               499 374];
        four_points_is.R{2} = [133 90;
                               42 270;
                               403 161;
                               314 392];
        
        % Board 3
        four_points_is.L{3} = [189 130;
                               181 329;
                               470 112;
                               475 338];
        four_points_is.R{3} = [59 149;
                               48 337;
                               308 120;
                               305 353];
               
        % Refine
        for i = 1:length(cb_imgs.L)
            four_points_is.L{i} = alg.refine_points(four_points_is.L{i}, ...
                                                    cb_imgs.L(i), ...
                                                    alg.homography(four_points_w,four_points_is.L{i}), ...
                                                    cb_config);  
            four_points_is.R{i} = alg.refine_points(four_points_is.R{i}, ...
                                                    cb_imgs.R(i), ...
                                                    alg.homography(four_points_w,four_points_is.R{i}), ...
                                                    cb_config);   
                                                    
            util.debug_cb_refine_points(four_points_is.L{i},cb_imgs.L(i),alg.homography(four_points_w,four_points_is.L{i}),cb_config,subplot(4,2,2*i+1,'parent',f1));
            util.debug_cb_refine_points(four_points_is.R{i},cb_imgs.R(i),alg.homography(four_points_w,four_points_is.R{i}),cb_config,subplot(4,2,2*i+2,'parent',f1));
        end   
end

%% Perform zhang calibration on left and right
[A.L,distortion.L,rotations.L,translations.L,board_points_is.L] = alg.zhang_calibrate(cb_imgs.L,four_points_is.L,cb_config);
[A.R,distortion.R,rotations.R,translations.R,board_points_is.R] = alg.zhang_calibrate(cb_imgs.R,four_points_is.R,cb_config);

% Debug by reprojecting points
f2 = figure(2);
for i = 1:length(cb_imgs.L)
    util.debug_cb_points_disp(alg.apply_full_model(A.L,distortion.L,rotations.L{i},translations.L{i},alg.cb_points(cb_config)), ...
                              board_points_is.L{i}, ...
                              cb_imgs.L(i), ...
                              subplot(3,2,2*i-1,'parent',f2));
    util.debug_cb_points_disp(alg.apply_full_model(A.R,distortion.R,rotations.R{i},translations.R{i},alg.cb_points(cb_config)), ...
                              board_points_is.R{i}, ...
                              cb_imgs.R(i), ...
                              subplot(3,2,2*i,'parent',f2));
end

%% Perform stereo refinement

% Get linear guess for R_s
r = zeros(9*length(rotations.L),1);
R = zeros(9*length(rotations.L),9);
for i = 1:length(rotations.L)
    top = (i-1)*9+1;
    R(top:top+2,1:3) = rotations.L{i};
    R(top+3:top+5,4:6) = rotations.L{i};
    R(top+6:top+8,7:9) = rotations.L{i};
    r(top:top+8) = reshape(rotations.R{i},[],1);
end

% Get leasts squares approximation
R_s = reshape(pinv(R)*r,3,3);

% Get best rotation approximation
[U,~,V] = svd(R_s);
R_s = U*V';

% Get linear guess for t
t_s = pinv(vertcat(rotations.L{:})) * (vertcat(translations.R{:})-vertcat(translations.L{:}));

% Debug initial guesses
f3 = figure(3);
for i = 1:length(cb_imgs.L)
    util.debug_cb_points_disp(alg.apply_full_model(A.L,distortion.L,rotations.L{i},translations.L{i},alg.cb_points(cb_config)), ...
                              board_points_is.L{i}, ...
                              cb_imgs.L(i), ...
                              subplot(3,2,2*i-1,'parent',f3));
    util.debug_cb_points_disp(alg.apply_full_model(A.R,distortion.R,rotations.L{i}*R_s,rotations.L{i}*t_s+translations.L{i},alg.cb_points(cb_config)), ...
                              board_points_is.R{i}, ...
                              cb_imgs.R(i), ...
                              subplot(3,2,2*i,'parent',f3));
end

% Nonlinear refinement next
[A_nl,distortion_nl,rotations_nl,translations_nl] = alg.nonlinear_stereo_params(A,distortion,rotations,translations,R_s,t_s,board_points_is,cb_config);

% Debug refined parameters
f4 = figure(4);
for i = 1:length(cb_imgs.L)
    util.debug_cb_points_disp(alg.apply_full_model(A_nl.L,distortion_nl.L,rotations_nl.L{i},translations_nl.L{i},alg.cb_points(cb_config)), ...
                              board_points_is.L{i}, ...
                              cb_imgs.L(i), ...
                              subplot(3,2,2*i-1,'parent',f4));
    util.debug_cb_points_disp(alg.apply_full_model(A_nl.R,distortion_nl.R,rotations_nl.L{i}*R_s,rotations_nl.L{i}*t_s+translations_nl.L{i},alg.cb_points(cb_config)), ...
                              board_points_is.R{i}, ...
                              cb_imgs.R(i), ...
                              subplot(3,2,2*i,'parent',f4));
end