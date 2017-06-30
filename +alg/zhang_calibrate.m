function [A,distortion,rotations,translations,board_points_is] = zhang_calibrate(cb_imgs,four_points_is,cb_config)
    % Performs zhang's camera calibration given calibration board images,
    % four point boxes around calibration board images and the calibration
    % board config.
    %
    % Inputs:
    %   cb_imgs - class.img; calibration board images
    %   four_points_is - cell; cell array of four points
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   A - array; optimized A
    %   distortion - array; 4x1 array of optimized distortions (radial and 
    %   tangential) stored as: 
    %       [beta1; beta2; beta3; beta4]
    %   rotations - cell; optimized rotations
    %   translations - cell; optimized translations    
    %   board_points_is - cell; cell array of optimized calibration board
    %       points.

    % Get homographies for four points -----------------------------------%
    [board_points_w, four_points_w] = alg.cb_points(cb_config);

    homographies_four_points = {};
    for i = 1:length(cb_imgs)
        homographies_four_points{i} = alg.linear_homography(four_points_w,four_points_is{i}); %#ok<AGROW>
    end

    % Refine points ------------------------------------------------------%
    % Apply homography to pocints
    board_points_is = {};
    for i = 1:length(cb_imgs)
        board_points_is{i} = alg.apply_homography(homographies_four_points{i},board_points_w); %#ok<AGROW>
    end

    % Refine points
    for i = 1:length(cb_imgs)    
        board_points_is{i} = alg.refine_points(board_points_is{i}, ...
                                               cb_imgs(i), ...
                                               homographies_four_points{i}, ...
                                               cb_config); %#ok<AGROW>
    end

    % Get homographies using refined points ------------------------------%
    homographies = {};
    for i = 1:length(cb_imgs)
        homographies{i} = alg.linear_homography(board_points_w,board_points_is{i}); %#ok<AGROW>
    end

    % Get initial guess for intrinsic camera parameters using all homographies
    A = alg.linear_intrinsic_params(homographies);

    % Get initial guess for extrinsic camera parameters (R and t) per homography.
    rotations = {};
    translations = {};
    for i = 1:length(cb_imgs)
        [rotations{i}, translations{i}] = alg.linear_extrinsic_params(homographies{i},A); %#ok<AGROW>
    end

    % Nonlinear refinement -----------------------------------------------%
    [A,distortion,rotations,translations] = alg.nonlinear_params(A,rotations,translations,board_points_is,cb_config);
end

