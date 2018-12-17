function calib = stereo_calib_fp_dr(img_cbs, p_fp_p_dss, calib_config, intrin)
    % Performs stereo camera calibration using "four point distortion
    % refinement" method.
    %
    % Inputs:
    %   img_paths - struct;
    %       .L - util.img; Nx1 calibration board images
    %       .R - util.img; Nx1 calibration board images
    %   p_fp_p_dss - struct;
    %       .L - cell; Nx1 cell of four point boxes around the
    %           calibration board images
    %       .R - cell; Nx1 cell of four point boxes around the
    %           calibration board images
    %   calib_config -struct; struct returned by util.read_calib_config()
    %   intrin - struct; optional, if passed in intrinsics will not be
    %       optimized.
    %       .L - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %       .R - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %
    % Outputs:
    %   calib - struct;
    %       .L - struct; calibration for left camera
    %       .R - struct; calibration for right camera
    %       .R_s - array; 3x3 rotation matrix describing rotation from
    %           left to right camera
    %       .t_s - array; 3x1 translation vector describing translation
    %           from left to right camera
    %       .debug; struct;
    %           .params - array; raw parameter vector returned by
    %               alg.refine_stereo_params()
    %           .cov_params - array; covariance of parameters returned by
    %               alg.refine_stereo_params()

    util.verbose_disp('------------', 1, calib_config);
    util.verbose_disp('Performing stereo calibration with four point distortion refinement method...', 1, calib_config);

    % Get distortion function
    sym_p_p2p_p_d = eval(calib_config.sym_p_p2p_p_d);

    % If intrinsics are passed in, don't optimize for them
    num_params_d = alg.num_params_d(sym_p_p2p_p_d);
    if exist('intrin', 'var')
        a.L = alg.A2a(intrin.L.A);
        a.R = alg.A2a(intrin.R.A);
        d.L = intrin.L.d;
        d.R = intrin.R.d;
        optimization_type = 'extrinsic';
    else
        optimization_type = 'full';
    end

    % Get number of boards
    num_boards = numel(img_cbs.L);

    % Get function handle for distortion function
    f_p_p2p_p_d = matlabFunction(sym_p_p2p_p_d);

    % Get function handles for distortion function partial derivatives
    args_p_p2p_p_d = argnames(sym_p_p2p_p_d);
    for i = 1:numel(args_p_p2p_p_d)
        % Differentiate
        f_dp_p_d_dargs{i} = diff(sym_p_p2p_p_d, ...
                                 args_p_p2p_p_d(i)); %#ok<AGROW>
        % Convert to function handle
        f_dp_p_d_dargs{i} = matlabFunction(f_dp_p_d_dargs{i}); %#ok<AGROW>
    end

    % Perform single calibration on the left and right cameras -----------%

    util.verbose_disp('---------', 1, calib_config);
    util.verbose_disp('Calibrating left camera...', 1, calib_config);

    if exist('intrin', 'var')
        calib.L = alg.single_calib_fp_dr(img_cbs.L, ...
                                         p_fp_p_dss.L, ...
                                         calib_config, ...
                                         intrin.L);
    else
        calib.L = alg.single_calib_fp_dr(img_cbs.L, ...
                                         p_fp_p_dss.L, ...
                                         calib_config);
    end

    util.verbose_disp('---------', 1, calib_config);
    util.verbose_disp('Calibrating right camera...', 1, calib_config);

    if exist('intrin', 'var')
        calib.R = alg.single_calib_fp_dr(img_cbs.R, ...
                                         p_fp_p_dss.R, ...
                                         calib_config, ...
                                         intrin.R);
    else
        calib.R = alg.single_calib_fp_dr(img_cbs.R, ...
                                         p_fp_p_dss.R, ...
                                         calib_config);
    end

    % Repackage initial guesses and other parameters ---------------------%

    if ~exist('intrin', 'var')
        a.L = alg.A2a(calib.L.intrin.A);
        a.R = alg.A2a(calib.R.intrin.A);
        d.L = calib.L.intrin.d;
        d.R = calib.R.intrin.d;
    end
    Rs.L = {calib.L.extrin.R};
    Rs.R = {calib.R.extrin.R};
    ts.L = {calib.L.extrin.t};
    ts.R = {calib.R.extrin.t};
    p_cb_p_dss.L = {calib.L.extrin.p_cb_p_ds};
    p_cb_p_dss.R = {calib.R.extrin.p_cb_p_ds};
    cov_cb_p_dss.L = {calib.L.extrin.cov_cb_p_ds};
    cov_cb_p_dss.R = {calib.R.extrin.cov_cb_p_ds};
    idx_valids.L = {calib.L.extrin.idx_valid};
    idx_valids.R = {calib.R.extrin.idx_valid};

    % Get initial guesses for transform between left and right cameras ---%

    % Get least squares linear initial guess for R_s
    r = [];
    R = [];
    for i = 1:num_boards
        r = vertcat(r, Rs.R{i}(:)); %#ok<AGROW>
        R = vertcat(R, [Rs.L{i}(1, 1)*eye(3) Rs.L{i}(2, 1)*eye(3) Rs.L{i}(3, 1)*eye(3);
                        Rs.L{i}(1, 2)*eye(3) Rs.L{i}(2, 2)*eye(3) Rs.L{i}(3, 2)*eye(3);
                        Rs.L{i}(1, 3)*eye(3) Rs.L{i}(2, 3)*eye(3) Rs.L{i}(3, 3)*eye(3)]); %#ok<AGROW>
    end

    % Get least squares approximation
    R_s_init = reshape(alg.lscov_finite(R, r), 3, 3);
    R_s_init = alg.approx_rot(R_s_init); % Get best rotational approximation

    % Get least squares linear guess for t_s
    t = [];
    T = [];
    for i = 1:num_boards
        t = vertcat(t, ts.R{i}-Rs.R{i}*Rs.L{i}'*ts.L{i}); %#ok<AGROW>
        T = vertcat(T, eye(3)); %#ok<AGROW>
    end

    % Get least squares approximation
    t_s_init = alg.lscov_finite(T, t);

    % Perform nonlinear refinement of all parameters ---------------------%

    util.verbose_disp('---------', 1, calib_config);
    util.verbose_disp('Refining stereo parameters...', 1, calib_config);

    % Get transform that converts world points to pixel points and its
    % corresponding derivatives wrt homography
    switch calib_config.target_type
        case 'checker'
            % Use "point to point" method
            f_p_w2p_p = @(p, H)alg.apply_homography_p2p(p, H);
            f_dp_p_dh = @(p, H)alg.dp_dh_p2p(p, H);
        case 'circle'
            % Use "circle to ellipse" method
            f_p_w2p_p = @(p, H)alg.apply_homography_c2e(p, H, calib_config.circle_radius);
            f_dp_p_dh = @(p, H)alg.dp_dh_c2e(p, H, calib_config.circle_radius);
        otherwise
            error(['Unknown target type: "' calib_config.target_type '"']);
    end

    % Gather params
    num_params = 6+2*num_params_d+6*(num_boards+1);
    params = zeros(num_params, 1);
    % Intrinsics
    params(1:3) = a.L;
    params(4:3+num_params_d) = d.L;
    params(4+num_params_d:6+num_params_d) = a.R;
    params(7+num_params_d:6+2*num_params_d) = d.R;
    % Extrinsics
    for i = 1:num_boards
        params(7+2*num_params_d+6*(i-1):9+2*num_params_d+6*(i-1)) = alg.rot2euler(Rs.L{i});
        params(10+2*num_params_d+6*(i-1):12+2*num_params_d+6*(i-1)) = ts.L{i};
    end
    params(7+2*num_params_d+6*num_boards:9+2*num_params_d+6*num_boards) = alg.rot2euler(R_s_init);
    params(10+2*num_params_d+6*num_boards:12+2*num_params_d+6*num_boards) = t_s_init;

    % Refine params
    if calib_config.apply_covariance_optimization
        [params, cov_params] = alg.refine_stereo_params(params, ...
                                                       p_cb_p_dss, ...
                                                       idx_valids, ...
                                                       f_p_w2p_p, ...
                                                       f_dp_p_dh, ...
                                                       f_p_p2p_p_d, ...
                                                       f_dp_p_d_dargs, ...
                                                       optimization_type, ...
                                                       calib_config, ...
                                                       cov_cb_p_dss);
    else
        [params, cov_params] = alg.refine_stereo_params(params, ...
                                                       p_cb_p_dss, ...
                                                       idx_valids, ...
                                                       f_p_w2p_p, ...
                                                       f_dp_p_dh, ...
                                                       f_p_p2p_p_d, ...
                                                       f_dp_p_d_dargs, ...
                                                       optimization_type, ...
                                                       calib_config);
    end

    % Parse params
    a.L = params(1:3);
    d.L = params(4:3+num_params_d);
    a.R = params(4+num_params_d:6+num_params_d);
    d.R = params(7+num_params_d:6+2*num_params_d);
    for i = 1:num_boards
        Rs.L{i} = alg.euler2rot(params(7+2*num_params_d+6*(i-1):9+2*num_params_d+6*(i-1)));
        ts.L{i} = params(10+2*num_params_d+6*(i-1):12+2*num_params_d+6*(i-1));
    end
    R_s = alg.euler2rot(params(7+2*num_params_d+6*num_boards:9+2*num_params_d+6*num_boards));
    t_s = params(10+2*num_params_d+6*num_boards:12+2*num_params_d+6*num_boards);
    for i = 1:num_boards
        Rs.R{i} = R_s*Rs.L{i};
        ts.R{i} = R_s*ts.L{i}+t_s;
    end

    % Print params
    util.verbose_disp('------', 1, calib_config);
    util.verbose_disp('Stereo intrinsic params (+- 3*sigma):', 1, calib_config);
    util.verbose_disp('  -Camera (L):                       -Camera (R): ', 1, calib_config);
    print_param('alpha', 1             , params, cov_params, '  '   , calib_config);
    print_param('alpha', 4+num_params_d, params, cov_params, newline, calib_config);
    print_param('x_o',   2             , params, cov_params, '  '   , calib_config);
    print_param('x_o',   5+num_params_d, params, cov_params, newline, calib_config);
    print_param('y_o',   3             , params, cov_params, '  '   , calib_config);
    print_param('y_o',   6+num_params_d, params, cov_params, newline, calib_config);
    util.verbose_disp('  -Distortion (L):                   -Distortion (R): ', 1, calib_config);
    for i = 1:num_params_d
        print_param(char(args_p_p2p_p_d(i+5)), i+3,              params, cov_params, '  '   , calib_config);
        print_param(char(args_p_p2p_p_d(i+5)), i+6+num_params_d, params, cov_params, newline, calib_config);
    end
    util.verbose_disp('Stereo extrinsic params (+- 3*sigma):', 1, calib_config);
    util.verbose_disp('  -Rotation (L => R):', 1, calib_config);
    print_param('theta_x', 7+2*num_params_d+6*num_boards,  params, cov_params, newline, calib_config);
    print_param('theta_y', 8+2*num_params_d+6*num_boards,  params, cov_params, newline, calib_config);
    print_param('theta_z', 9+2*num_params_d+6*num_boards,  params, cov_params, newline, calib_config);
    util.verbose_disp('  -Translation (L => R):', 1, calib_config);
    print_param('x',       10+2*num_params_d+6*num_boards, params, cov_params, newline, calib_config);
    print_param('y',       11+2*num_params_d+6*num_boards, params, cov_params, newline, calib_config);
    print_param('z',       12+2*num_params_d+6*num_boards, params, cov_params, newline, calib_config);

    % Repackage outputs --------------------------------------------------%
    calib.L.intrin.A = alg.a2A(a.L);
    calib.L.intrin.d = d.L;
    calib.R.intrin.A = alg.a2A(a.R);
    calib.R.intrin.d = d.R;
    for i = 1:num_boards
        calib.L.extrin(i).R = Rs.L{i};
        calib.L.extrin(i).t = ts.L{i};
        calib.R.extrin(i).R = Rs.R{i};
        calib.R.extrin(i).t = ts.R{i};
    end
    calib.R_s = R_s;
    calib.t_s = t_s;
    calib.debug.params = params;
    calib.debug.cov_params = cov_params;
end

function print_param(s_param, idx, params, cov_params, suffix, calib_config)
    util.verbose_fprintf([pad(['    ' s_param ': '], 13) sprintf('% 9.4f', params(idx)) ' +- ' sprintf('% 6.4f', 3*sqrt(cov_params(idx, idx))) suffix], 1, calib_config);
end
