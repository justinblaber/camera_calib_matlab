function [A,distortion,rotations,translations,board_points_ps,homographies_refine] = single_calibrate(cb_imgs,four_points_ps,cal_config)
    % Performs camera calibration (mostly from Zhang's paper, some stuff
    % adapted from Bouguet's toolbox, and some stuff I've added myself) 
    % given calibration board images, four point boxes around the 
    % calibration board images, and the calibration board config.
    %
    % Inputs:
    %   cb_imgs - class.img; calibration board images
    %   four_points_ps - cell; cell array of four points
    %   cal_config - struct; this is the struct returned by
    %       util.load_cal_config()
    %
    % Outputs:
    %   A - array; optimized camera matrix
    %   distortion - array; 4x1 array of optimized distortions (radial and 
    %   tangential) stored as: 
    %       [beta1; beta2; beta3; beta4]  
    %   rotations - cell; optimized rotations
    %   translations - cell; optimized translations  
    %   board_points_ps - cell; cell array of optimized subpixel 
    %       calibration board points.
    %   homographies_refine - cell; cell array of homographies used for
    %       subpixel checkerboard corner refinement
                                                             
    disp('---');
    disp('Performing single calibration...');       
    
    % Get calibration board points in world coordinates
    [board_points_w, four_points_w] = alg.cb_points(cal_config);
    
    % Initialize homographies using four points in pixel coordinates -----%
    homographies = {};
    for i = 1:length(cb_imgs)
        homographies{i} = alg.homography(four_points_w, ...
                                         four_points_ps{i}, ...
                                         cal_config); %#ok<AGROW>
    end

    % Get sub-pixel board points in pixel coordinates --------------------%
    % Use homography to initialize board points in pixel coordinates
    board_points_ps = {};
    for i = 1:length(cb_imgs)
        board_points_ps{i} = alg.apply_homography(homographies{i}, ...
                                                  board_points_w); %#ok<AGROW>
    end

    % Refine points
    for i = 1:length(cb_imgs)    
        board_points_ps{i} = alg.refine_points(board_points_ps{i}, ...
                                               cb_imgs(i), ...
                                               homographies{i}, ...
                                               cal_config); %#ok<AGROW>
    end
    homographies_refine = homographies;

    % Update homographies using refined points ---------------------------%
    for i = 1:length(cb_imgs)
        homographies{i} = alg.homography(board_points_w, ...
                                         board_points_ps{i}, ...
                                         cal_config); %#ok<AGROW>
    end

    % Get initial guess for intrinsic camera parameters using all homographies
    A = alg.init_intrinsic_params(homographies, ...
                                  cb_imgs(1).get_width(), ...
                                  cb_imgs(1).get_height()); 
                              
    % Display initial intrinsic params for debugging purposes
    disp('---');
    disp('Initial intrinsic params: ')
    disp(A)

    % Get initial guess for extrinsic camera parameters (R and t) per homography
    rotations = {};
    translations = {};
    for i = 1:length(cb_imgs)
        [rotations{i}, translations{i}] = alg.init_extrinsic_params(homographies{i}, ...
                                                                    A, ...
                                                                    board_points_ps{i}, ...
                                                                    cal_config); %#ok<AGROW>
    end

    % Perform nonlinear refinement of all parameters ---------------------%
    % Initialize distortion parameters to zero
    distortion = zeros(4,1);
    
    % Perform full optimization
    disp('---');
    disp('Refining full single parameters...');
    [A,distortion,rotations,translations] = alg.refine_single_params(A, ... 
                                                                     distortion, ...
                                                                     rotations, ...
                                                                     translations, ...
                                                                     board_points_ps, ...
                                                                     'full', ...
                                                                     cal_config);   
                                                    
    disp('---');             
    disp('Refined intrinsic params: ')
    disp(A)
end
