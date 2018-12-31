function jacob = dp_cb_p_d_dintrinsic(p_cb_ws, f_p_cb_w2p_cb_p, f_dp_cb_p_dh, R, t, f_dp_p_d_dargs, a, d)
    % Returns jacobian of calibration board distorted pixel points wrt
    % intrinsics.
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
    %
    % Outputs:
    %   jacob - array; 2*Nx(3+M) array.
    %       Format of jacobian is:
    %
    %                   dalpha dx_o dy_o dd_1, ..., dd_M
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

    % 1) alpha: [dp_cb_p_d/dh]*[dh/dalpha] + [dp_cb_p_d/dalpha]
    dh_dalpha = [R(1, 1); R(2, 1); 0; R(1, 2); R(2, 2); 0; t(1); t(2); 0];
    dp_cb_p_d_dalpha = dp_cb_p_d_dh*dh_dalpha + alg.dp_p_d_darg(p_cb_ps, f_dp_p_d_dargs{3}, a, d);

    % 2) x_o: [dp_cb_p_d/dh]*[dh/dx_o] + [dp_cb_p_d/dx_o]
    dh_dx_o = [R(3, 1); 0; 0; R(3, 2); 0; 0; t(3); 0; 0];
    dp_cb_p_d_dx_o = dp_cb_p_d_dh*dh_dx_o + alg.dp_p_d_darg(p_cb_ps, f_dp_p_d_dargs{4}, a, d);

    % 3) y_o: [dp_cb_p_d/dh]*[dh/dy_o] + [dp_cb_p_d/dy_o]
    dh_dy_o = [0; R(3, 1); 0; 0; R(3, 2); 0; 0; t(3); 0];
    dp_cb_p_d_dy_o = dp_cb_p_d_dh*dh_dy_o + alg.dp_p_d_darg(p_cb_ps, f_dp_p_d_dargs{5}, a, d);

    % 4) d: [dp_cb_p_d/dd] - assumes p_cb_p has no dependence on these parameters
    num_params_d = alg.num_params_d(f_dp_p_d_dargs{1});
    dp_cb_p_d_dd = zeros(2*size(p_cb_ws, 1), num_params_d);
    for i = 1:num_params_d
        dp_cb_p_d_dd(:, i) = alg.dp_p_d_darg(p_cb_ps, f_dp_p_d_dargs{i+5}, a, d);
    end

    % Form jacobian
    jacob = [dp_cb_p_d_dalpha dp_cb_p_d_dx_o dp_cb_p_d_dy_o dp_cb_p_d_dd];
end
