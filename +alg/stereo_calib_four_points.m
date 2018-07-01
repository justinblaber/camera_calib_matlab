function [calib,R_s,t_s] = stereo_calib_four_points(cb_imgs,four_points_ps,calib_config,intrin)
    % Performs stereo calibration (mostly from Zhang's paper, some stuff
    % adapted from Bouguet's toolbox, and some stuff I've added myself) 
    % given calibration board images for the left and right camera, four 
    % point boxes around the calibration board images for the left and
    % right camera, and the calibration config.
    %
    % Inputs:
    %   cb_imgs - struct; contains:
    %       .L and .R - class.img; Mx1 calibration board images
    %   four_points_ps - struct; contains:
    %       .L and .R - cell; Mx1 cell array of four points
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %   intrin - struct; optional input containing intrinsic parameters; if
    %       this is passed in, then only extrinsic parameters are optimized 
    %       contains:
    %           .L and .R - struct containing intrinsic params
    %
    % Outputs:
    %   calib - struct; contains:
    %       .L and .R - struct; contains:
    %           .config - struct; this is the struct returned by
    %               util.read_calib_config()
    %           .intrin.A - array; optimized camera matrix
    %           .intrin.distortion - array; optimized distortions (radial 
    %               and tangential) stored as: 
    %               [beta1; beta2; beta3; beta4]  
    %           .extrin(i).cb_img - class.img; ith calibration board image
    %           .extrin(i).rotation - array; ith optimized rotation
    %           .extrin(i).translation - array; ith optimized translation
    %           .extrin(i).four_points_p - array; ith array of four points
    %               around calibration board in pixel coordinates.
    %           .extrin(i).board_points_p - array; ith array of optimized 
    %               subpixel calibration board points in pixel coordinates.
    %           .extrin(i).debug.homography_refine - array; ith homography 
    %               used for subpixel target refinement.
    %   R_s - array; optimized rotation describing rotation from the left 
    %       camera to the right camera
    %   t_s - array; optimized translation describing translation from the 
    %       left camera to the right camera
                                               
    disp('--------------------------------------------'); 
    disp('Performing stereo calibration...');     
        
    % Get number of boards
    num_boards = length(cb_imgs.L);
    
    % Perform single calibration on the left and right camera ------------%                                                                
    disp('---------');
    disp('Calibrating left camera...');   
    
    if exist('intrin','var')
        calib.L = alg.single_calib_four_points(cb_imgs.L, ...
                                               four_points_ps.L, ...
                                               calib_config, ...
                                               intrin.L);
    else
        calib.L = alg.single_calib_four_points(cb_imgs.L, ...
                                               four_points_ps.L, ...
                                               calib_config);
    end
        
                                                                  
    disp('---------');
    disp('Calibrating right camera...'); 
    
    if exist('intrin','var')
        calib.R = alg.single_calib_four_points(cb_imgs.R, ...
                                               four_points_ps.R, ...
                                               calib_config, ...
                                               intrin.R);
    else
        calib.R = alg.single_calib_four_points(cb_imgs.R, ...
                                               four_points_ps.R, ...
                                               calib_config);
    end
                           
    % Repackage initial guesses and other parameters ---------------------%
    A.L = calib.L.intrin.A;
    A.R = calib.R.intrin.A;
    distortion.L = calib.L.intrin.distortion;
    distortion.R = calib.R.intrin.distortion;
    rotations.L = {calib.L.extrin.rotation};
    rotations.R = {calib.R.extrin.rotation};
    translations.L = {calib.L.extrin.translation};
    translations.R = {calib.R.extrin.translation};
    board_points_ps.L = {calib.L.extrin.board_points_p};
    board_points_ps.R = {calib.R.extrin.board_points_p};
                               
    % Perform stereo refinement ------------------------------------------%
    % Get least squares linear initial guess for R_s
    r = [];
    R = [];
    for i = 1:num_boards
        r = vertcat(r,rotations.R{i}(:)); %#ok<AGROW>
        R = vertcat(R,[rotations.L{i}(1,1)*eye(3) rotations.L{i}(2,1)*eye(3) rotations.L{i}(3,1)*eye(3);
                       rotations.L{i}(1,2)*eye(3) rotations.L{i}(2,2)*eye(3) rotations.L{i}(3,2)*eye(3);
                       rotations.L{i}(1,3)*eye(3) rotations.L{i}(2,3)*eye(3) rotations.L{i}(3,3)*eye(3)]); %#ok<AGROW>
    end

    % Get least squares approximation
    R_s_init = reshape(mldivide(R,r),3,3);

    % R_s is not necessarily orthogonal, so get the best rotational 
    % approximation.
    R_s_init = alg.approx_rot(R_s_init);

    % Get least squares linear guess for t
    t = [];
    T = [];
    for i = 1:num_boards
        t = vertcat(t,translations.R{i}-rotations.R{i}*rotations.L{i}'*translations.L{i}); %#ok<AGROW>
        T = vertcat(T,eye(3)); %#ok<AGROW>
    end

    % Get least squares approximation
    t_s_init = mldivide(T,t);

    % Perform nonlinear refinement of all parameters ---------------------%
    
    if exist('intrin','var')
        optimization_type = 'extrinsic';
    else
        optimization_type = 'full';
    end
    
    disp('---');
    disp('Refining full stereo parameters...');
    [A,distortion,rotations,translations,R_s,t_s] = alg.refine_stereo_params(A, ...
                                                                             distortion, ...
                                                                             rotations, ...
                                                                             translations, ...
                                                                             board_points_ps, ...
                                                                             R_s_init, ...
                                                                             t_s_init, ....
                                                                             optimization_type, ...
                                                                             calib_config);  
    
    disp('---');                           
    disp('Stereo refined camera matrix (left): ');
    disp(A.L);
    disp('Stereo refined distortions (left): ');
    disp(distortion.L');
    disp('Stereo refined camera matrix (right): ');
    disp(A.R);
    disp('Stereo refined distortions (right): ');
    disp(distortion.R');
    disp('Initial t_s: ');
    disp(t_s_init');
    disp('Refined t_s: ');
    disp(t_s');
    disp('Initial R_s: ');
    disp(R_s_init);
    disp('Refined R_s: ');
    disp(R_s);
    
    % Repackage outputs --------------------------------------------------%
    % Only need to redo outputs of refine_stereo_params
    calib.L.intrin.A = A.L;
    calib.R.intrin.A = A.R;
    calib.L.intrin.distortion = distortion.L;
    calib.R.intrin.distortion = distortion.R;
    for i = 1:num_boards
        calib.L.extrin(i).rotation = rotations.L{i};
        calib.R.extrin(i).rotation = rotations.R{i};
        calib.L.extrin(i).translation = translations.L{i};
        calib.R.extrin(i).translation = translations.R{i};
    end
end
