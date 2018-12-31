function jacob = dp_cb_p_d_dextrinsic(p_cb_ws, f_p_cb_w2p_cb_p, f_dp_cb_p_dh, R, t, f_dp_p_d_dargs, a, d, drt_dm)
    % Returns jacobian of calibration board distorted pixel points wrt
    % extrinsics.
    %
    % Inputs:
    %   p_cb_ws - array; Nx2 array of calibration board world points
    %   f_p_cb_w2p_cb_p - function handle; function which transforms
    %       calibration world points to calibration pixel points
    %   f_dp_cb_p_dh - function handle; derivative of f_p_cb_w2p_cb_p wrt
    %      homography parameters.
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector
    %   f_dp_p_d_dargs - function handle; derivative of p_p2p_p_d wrt its
    %       input arguments.
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    %   d - array; Mx1 array of distortion coefficients
    %   drt_dm - array; 12x6 array containing jacobian of "rt" wrt
    %       extrinsics.
    %
    % Outputs:
    %   jacob - array; 2*Nx6 array.
    %       Format of jacobian is:
    %
    %                   dtheta_x dtheta_y dtheta_z t_x t_y t_z
    %       dx_cb_p_d_1
    %       dy_cb_p_d_1
    %          .
    %          .
    %          .
    %       dx_cb_p_d_N
    %       dy_cb_p_d_N

    % Get homography
    H = alg.ARt2H(alg.a2A(a), R, t);

    % Get calibration board pixel points
    p_cb_ps = f_p_cb_w2p_cb_p(p_cb_ws, H);

    % Get dp_cb_p_d/dh
    dp_cb_p_dh = f_dp_cb_p_dh(p_cb_ws, H);
    dp_cb_p_d_dp_cb_p = alg.dp_p_d_dp_p(p_cb_ps, f_dp_p_d_dargs{1}, f_dp_p_d_dargs{2}, a, d);
    dp_cb_p_d_dh = dp_cb_p_d_dp_cb_p*dp_cb_p_dh;

    % Get dh/drt
    dh_drt = alg.dh_drt(alg.a2A(a));

    % Form jacobian
    jacob = dp_cb_p_d_dh*dh_drt*drt_dm;
end
