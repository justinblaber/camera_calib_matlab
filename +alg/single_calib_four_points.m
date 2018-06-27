function calib = single_calib_four_points(cb_imgs,four_points_ps,calib_config)
    % Performs camera calibration (mostly from Zhang's paper, some stuff
    % adapted from Bouguet's toolbox, and some stuff I've added myself) 
    % given calibration board images, four point boxes around the 
    % calibration board images, and the calibration config.
    %
    % Inputs:
    %   cb_imgs - class.img; Mx1 calibration board images
    %   four_points_ps - cell; Mx1 cell array of four points
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   calib - struct; contains:
    %       .config - struct; this is the struct returned by
    %           util.read_calib_config()
    %       .intrin.A - array; optimized camera matrix
    %       .intrin.distortion - array; optimized distortions (radial and
    %           tangential) stored as: 
    %           [beta1; beta2; beta3; beta4]  
    %       .extrin(i).cb_img - class.img; ith calibration board image
    %       .extrin(i).rotation - array; ith optimized rotation
    %       .extrin(i).translation - array; ith optimized translation
    %       .extrin(i).four_points_p - array; ith array of four points
    %           around calibration board in pixel coordinates.
    %       .extrin(i).board_points_p - array; ith array of optimized 
    %           subpixel calibration board points in pixel coordinates.
    %       .extrin(i).debug.homography_refine - array; ith homography used
    %           for subpixel checkerboard corner refinement.
                                                             
    disp('---');
    disp('Performing single calibration...');       
    
    % Get calibration board points in world coordinates
    [board_points_w, four_points_w] = alg.cb_points(calib_config);
    
    % Initialize homographies using four points in pixel coordinates -----%
    homographies = {};
    for i = 1:length(cb_imgs)
        homographies{i} = alg.homography(four_points_w, ...
                                         four_points_ps{i}, ...
                                         calib_config); %#ok<AGROW>
    end

    % Get sub-pixel board points -----------------------------------------%
    % Use homography to initialize
    board_points_ps = {};
    for i = 1:length(cb_imgs)
        board_points_ps{i} = alg.apply_homography(homographies{i}, ...
                                                  board_points_w); %#ok<AGROW>
    end

    % Refine points    
    disp('---');
    for i = 1:length(cb_imgs)    
        t = tic;
        fprintf('Refining "%s" points for: %s. ',calib_config.calibration_target,cb_imgs(i).get_path());
        
        switch calib_config.calibration_target
            case 'checker'
                board_points_ps{i} = alg.refine_checkers(board_points_ps{i}, ...
                                                         cb_imgs(i).get_gs(), ...
                                                         homographies{i}, ...
                                                         calib_config); %#ok<AGROW>
            otherwise
                error(['Calibration target: "' calib_config.calibration_target '" is not supported.']);
        end
        
        time = toc(t);
        fprintf('Time ellapsed: %f seconds.\n',time);
    end
    % Store for debugging purposes
    homographies_refine = homographies;

    % Update homographies using refined points ---------------------------%
    for i = 1:length(cb_imgs)
        homographies{i} = alg.homography(board_points_w, ...
                                         board_points_ps{i}, ...
                                         calib_config); %#ok<AGROW>
    end

    % Get initial guess for intrinsic camera parameters using all homographies
    A_init = alg.init_intrinsic_params(homographies, ...
                                       cb_imgs(1).get_width(), ...
                                       cb_imgs(1).get_height()); 
                
    % Get initial guess for extrinsic camera parameters (R and t) per homography
    rotations = {};
    translations = {};
    for i = 1:length(cb_imgs)
        [rotations{i}, translations{i}] = alg.init_extrinsic_params(homographies{i}, ...
                                                                    A_init, ...
                                                                    board_points_ps{i}, ...
                                                                    calib_config); %#ok<AGROW>
    end

    % Perform nonlinear refinement of all parameters ---------------------%
    % Initialize distortion parameters to zero
    distortion = zeros(4,1);
    
    % Perform full optimization
    disp('---');
    disp('Refining full single parameters...');
    [A,distortion,rotations,translations] = alg.refine_single_params(A_init, ... 
                                                                     distortion, ...
                                                                     rotations, ...
                                                                     translations, ...
                                                                     board_points_ps, ...
                                                                     'full', ...
                                                                     calib_config);   
                                                                 
    % TODO: possibly add frontal refinement here
                              
    disp('---');
    disp('Initial intrinsic params: ')
    disp(A_init)    
    disp('---');             
    disp('Refined intrinsic params: ');
    disp(A);
    
    % Package outputs ----------------------------------------------------%
    calib.config = calib_config;
    calib.intrin.A = A;
    calib.intrin.distortion = distortion;
    for i = 1:length(cb_imgs)
        calib.extrin(i).cb_img = cb_imgs(i);
        calib.extrin(i).rotation = rotations{i};
        calib.extrin(i).translation = translations{i};
        calib.extrin(i).four_points_p = four_points_ps{i};
        calib.extrin(i).board_points_p = board_points_ps{i};
        calib.extrin(i).debug.homography_refine = homographies_refine{i};
    end
end
