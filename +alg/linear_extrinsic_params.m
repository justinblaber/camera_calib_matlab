function [R,t] = linear_extrinsic_params(homography,A)
    % This will compute rotation and translation from the homography given
    % the intrinsic parameters matrix A.
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
    %
    % Outputs:
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector
    %
    % Note that outputs are an approximation, so if you recompute the
    % homography with A*[R(:,1) R(:,2) t], the homographies will be
    % slightly different. Must use non-linear refinement after this to get
    % a better approximation of A, R and t.
    
    h1 = homography(:,1);
    h2 = homography(:,2);
    h3 = homography(:,3);
    A_inv = A^-1;
    
    % Compute scaling factors
    lambda1 = norm(A_inv*h1);
    lambda2 = norm(A_inv*h2);
    
    % Compute rotation
    r1 = A_inv*h1/lambda1;
    r2 = A_inv*h2/lambda2;
    r3 = cross(r1,r2);
    
    % Get nearest orthogonal rotation matrix
    R = [r1 r2 r3];
    [U,~,V] = svd(R);
    R = U*V';
    
    % Compute translation - use average of both lambdas to normalize
    t = A_inv*h3/((lambda1+lambda2)/2);
end