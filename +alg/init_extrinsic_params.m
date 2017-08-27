function [R,t] = init_extrinsic_params(homography,A,board_points_i,cb_config)
    % This will compute initial guesses for rotation and translation of a 
    % single calibration board. It will first compute an initial linear 
    % guess and then perform non-linear refinement.
    %
    % Inputs:
    %   homography - array; 3x3 homography array. Note that homography(3,3)
    %   should be positive (if computed through alg.linear_homography(), it
    %   will be 1), which guarantees t(3) (translation in the z direction)
    %   is positive.
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]
    %   board_points_i - array; Nx2 array of calibration board points in
    %       image coordinates.
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector
    
    % Get initial guess --------------------------------------------------%
    % Remove intrinsics from homography
    H_bar = A^-1*homography;
    
    % Compute scaling factors
    lambda1 = norm(H_bar(:,1));
    lambda2 = norm(H_bar(:,2));
    
    % Compute initial guess for rotation matrix
    r1 = H_bar(:,1)./lambda1;
    r2 = H_bar(:,2)./lambda2;
    r3 = cross(r1,r2);
    
    % Initial guess is not necessarily orthogonal, so get the best 
    % rotational approximation.
    R = alg.approx_rot([r1 r2 r3]);
    
    % Compute translation - use average of both lambdas to normalize
    t = H_bar(:,3)./mean([lambda1 lambda2]);
        
    % Perform non-linear refinement --------------------------------------%    
    disp('--------------------------------------------');
    disp('Refining initial extrinsic parameters...');
    [~,~,rotations,translations] = alg.refine_single_params(A, ...  
                                                            [0 0 0 0], ... % set distortions to zero
                                                            {R}, ...
                                                            {t}, ...
                                                            {board_points_i}, ...
                                                            'extrinsic', ...
                                                            cb_config);
    R = rotations{1};
    t = translations{1};
end