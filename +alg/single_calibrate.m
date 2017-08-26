function [rotations,translations,A,distortion,board_points_is] = single_calibrate(cb_imgs,four_points_is,cb_config)
    % Performs camera calibration (mostly from Zhang's paper, some stuff
    % adapted from Bouguet's toolbox, and some stuff I've added myself) 
    % given calibration board images, four point boxes around the 
    % calibration board images, and the calibration board config.
    %
    % Inputs:
    %   cb_imgs - class.img; calibration board images
    %   four_points_is - cell; cell array of four points
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   rotations - cell; optimized rotations
    %   translations - cell; optimized translations  
    %   A - array; optimized camera matrix
    %   distortion - array; 4x1 array of optimized distortions (radial and 
    %   tangential) stored as: 
    %       [beta1; beta2; beta3; beta4]  
    %   board_points_is - cell; cell array of optimized subpixel 
    %       calibration board points.

    % Get calibration board points in world coordinates
    [board_points_w, four_points_w] = alg.cb_points(cb_config);
    
    % Initialize homographies using four points in image coordinates -----%
    homographies = {};
    for i = 1:length(cb_imgs)
        homographies{i} = alg.homography(four_points_w, ...
                                         four_points_is{i}, ...
                                         cb_config); %#ok<AGROW>
    end

    % Get sub-pixel board points in image coordinates --------------------%
    % Use homography to initialize board points in image coordinates
    board_points_is = {};
    for i = 1:length(cb_imgs)
        board_points_is{i} = alg.apply_homography(homographies{i}, ...
                                                  board_points_w); %#ok<AGROW>
    end

    % Refine points
    for i = 1:length(cb_imgs)    
        board_points_is{i} = alg.refine_points(board_points_is{i}, ...
                                               cb_imgs(i), ...
                                               homographies{i}, ...
                                               cb_config); %#ok<AGROW>
    end

    % Update homographies using refined points ---------------------------%
    for i = 1:length(cb_imgs)
        homographies{i} = alg.homography(board_points_w, ...
                                         board_points_is{i}, ...
                                         cb_config); %#ok<AGROW>
    end

    % Get initial guess for intrinsic camera parameters using all homographies
    A = alg.init_intrinsic_params(homographies, ...
                                  cb_imgs(1).get_width(), ...
                                  cb_imgs(1).get_height()); 
                              
    % Display initial intrinsic params for debugging purposes
    disp('Initial intrinsic params: ')
    disp(A)

    % Get initial guess for extrinsic camera parameters (R and t) per homography
    rotations = {};
    translations = {};
    for i = 1:length(cb_imgs)
        [rotations{i}, translations{i}] = alg.init_extrinsic_params(homographies{i}, ...
                                                                    A, ...
                                                                    board_points_is{i}, ...
                                                                    cb_config); %#ok<AGROW>
    end

    % Perform nonlinear refinement of all parameters ---------------------%
    % Initialize distortion parameters to zero
    distortion = [0 0 0 0];
    
    % Perform full optimization
    disp('--------------------------------------------');
    disp('Refining full parameters...');
    [rotations,translations,A,distortion] = alg.refine_single_params(rotations, ...
                                                                     translations, ...
                                                                     A, ... 
                                                                     distortion, ... % init distortion to zero
                                                                     board_points_is, ...
                                                                     'full', ...
                                                                     cb_config);      
end