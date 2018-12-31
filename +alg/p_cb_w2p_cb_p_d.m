function p_cb_p_ds = p_cb_w2p_cb_p_d(p_cb_ws, f_p_cb_w2p_cb_p, R, t, f_p_p2p_p_d, a, d)
    % Transforms calibration board world points to calibration board
    % distorted pixel points.
    %
    % Inputs:
    %   p_cb_ws - array; Nx2 array of calibration board world points
    %   f_p_cb_w2p_cb_p - function handle; function which transforms
    %       calibration world points to calibration pixel points
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector
    %   f_p_p2p_p_d - function handle; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    %   d - array; Mx1 array of distortion coefficients
    %
    % Outputs:
    %   p_cb_p_ds - array; Nx2 array of calibration board distorted pixel
    %       points

    % Get homography
    H = alg.ARt2H(alg.a2A(a), R, t);

    % Get calibration board pixel points
    p_cb_ps = f_p_cb_w2p_cb_p(p_cb_ws, H);

    % Get calibration board distorted pixel points.
    p_cb_p_ds = alg.p_p2p_p_d(p_cb_ps, f_p_p2p_p_d, a, d);
end
