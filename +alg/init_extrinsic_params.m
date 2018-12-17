function [R, t] = init_extrinsic_params(H, A)
    % This will compute initial guesses for the rotation and translation
    % of a single calibration board image using linear least squares.
    %
    % Inputs:
    %   H - array; 3x3 homography array. Note that homography(3, 3) should
    %       be positive, which guarantees t(3) (translation in the z
    %       direction) is positive.
    %   A - array; 3x3 camera matrix
    %
    % Outputs:
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector

    % Remove intrinsics from homography
    H_bar = A^-1*H;

    % Compute scaling factors
    lambda1 = norm(H_bar(:, 1));
    lambda2 = norm(H_bar(:, 2));

    % Compute initial guess for rotation matrix
    r1 = H_bar(:, 1)./lambda1;
    r2 = H_bar(:, 2)./lambda2;
    r3 = cross(r1, r2);

    % Initial guess is not necessarily orthogonal, so get the best
    % rotational approximation.
    R = alg.approx_rot([r1 r2 r3]);

    % Compute translation - use average of both lambdas to normalize
    t = H_bar(:, 3)./mean([lambda1 lambda2]);
end
