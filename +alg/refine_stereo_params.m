function [params, cov_params] = refine_stereo_params(params, p_cb_p_dss, idx_valids, f_p_w2p_p, f_dp_p_dh, f_p_p2p_p_d, f_dp_p_d_dargs, optimization_type, opts, cov_cb_p_dss)
    % This will compute nonlinear refinement of intrinsic and extrinsic
    % camera parameters.
    %
    % Inputs:
    %   params - array; (6+2*M+6*(N+1))x1 array, where M is the number of
    %       distortion parameters and N is the number of calibration
    %       boards. Contains:
    %           [alpha_L; x_L_o; y_L_o; d_L_1; ... d_L_M; ...
    %            alpha_R; x_R_o; y_R_o; d_R_1; ... d_R_M; ...
    %            theta_L_x1; theta_L_y1; theta_L_z1; t_L_x1; t_L_y1; t_L_z1; ...
    %            theta_L_xN; theta_L_yN; theta_L_zN; t_L_xN; t_L_yN; t_L_zN; ...
    %            theta_s_x; theta_s_y; theta_s_z; t_s_x; t_s_y; t_s_z]
    %   p_cb_p_dss - struct;
    %       .L - cell; Nx1 cell array of calibration board points in
    %           distorted pixel coordinates for left camera
    %       .R - cell; Nx1 cell array of calibration board points in
    %           distorted pixel coordinates for right camera
    %   idx_valids - struct;
    %       .L - cell; Nx1 cell array of "valid" calibration board
    %           points for left camera
    %       .R - cell; Nx1 cell array of "valid" calibration board
    %           points for right camera
    %   f_p_w2p_p - function handle; function which transforms world
    %     coordinates to pixel coordinates
    %   f_dp_p_dh - function handle; derivative of p_w2p_p wrt homography
    %       parameters.
    %   f_p_p2p_p_d - function handle; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.
    %   f_dp_p_d_dargs - function handle; derivative of p_p2p_p_d wrt its
    %       input arguments.
    %   optimization_type - string; describes type of optimization
    %   opts - struct;
    %       .num_targets_height - int; number of targets in the "height"
    %           dimension
    %       .num_targets_width - int; number of targets in the "width"
    %           dimension
    %       .target_spacing - scalar; space between targets
    %       .refine_stereo_params_lambda_init - scalar; initial lambda for
    %           Levenberg-Marquardt method
    %       .refine_stereo_params_lambda_factor - scalar; scaling factor
    %           for lambda
    %       .refine_stereo_params_it_cutoff - int; max number of iterations
    %           performed for refinement of camera parameters
    %       .refine_stereo_params_norm_cutoff - scalar; cutoff for norm of
    %           difference of parameter vector for refinement of camera
    %           parameters.
    %       .verbose - int; level of verbosity
    %   cov_cb_p_dss - struct; optional
    %       .L - cell; Nx1 cell array of covariances of calibration board
    %           points in distorted pixel coordinates of left camera.
    %       .R - cell; Nx1 cell array of covariances of calibration board
    %           points in distorted pixel coordinates of right camera.
    %
    % Outputs:
    %   params - array; (6+2*M+6*(N+1))x1 array, where M is the number of
    %       distortion parameters and N is the number of calibration
    %       boards. Contains:
    %           [alpha_L; x_L_o; y_L_o; d_L_1; ... d_L_M; ...
    %            alpha_R; x_R_o; y_R_o; d_R_1; ... d_R_M; ...
    %            theta_L_x1; theta_L_y1; theta_L_z1; t_L_x1; t_L_y1; t_L_z1; ...
    %            theta_L_xN; theta_L_yN; theta_L_zN; t_L_xN; t_L_yN; t_L_zN; ...
    %            theta_s_x; theta_s_y; theta_s_z; t_s_x; t_s_y; t_s_z]
    %   cov_params - array; (6+2*M+6*(N+1))x(6+2*M+6*(N+1)) array of
    %       covariances of intrinsic and extrinsic parameters

    % Get board points in world coordinates
    p_cb_ws = alg.p_cb_w(opts);

    % Get number of boards
    num_boards = numel(p_cb_p_dss.L);

    % Get number of distortion params
    num_params_d = alg.num_params_d(f_p_p2p_p_d);

    % Determine which parameters to update based on type
    idx_update = false(size(params));
    switch optimization_type
        case 'extrinsic'
            % Only update rotations and translations
            idx_update(7+2*num_params_d:end) = true;
        case 'full'
            % Update everything
            idx_update(1:end) = true;
        otherwise
            error(['Input type of: "' optimization_type '" was not recognized']);
    end

    % For single images, remove principle point from optimization
    if num_boards == 1
        idx_update(2:3) = false;
        idx_update(9:10) = false;
    end

    % Get "weight matrix"
    if exist('cov_cb_p_dss', 'var')
        % Do generalized least squares
        % Get "weight" matrix (inverse of covariance)
        cov = vertcat(cov_cb_p_dss.L{:}, cov_cb_p_dss.R{:});    % Concat
        cov = cov(vertcat(idx_valids.L{:}, idx_valids.R{:}));   % Apply valid indices
        cov = cellfun(@sparse, cov, 'UniformOutput', false);    % Make sparse
        cov = blkdiag(cov{:});                                  % Create full covariance matrix
        W = inv(cov);                                           % This might be slow...
    else
        % Identity weight matrix is just simple least squares
        W = speye(2*sum(vertcat(idx_valids.L{:}, idx_valids.R{:})));
    end

    % Perform Levenberg–Marquardt iteration(s)
    % Initialize lambda
    lambda = opts.refine_stereo_params_lambda_init;
    % Get initial cost
    cost = calc_cost(params, ...
                     p_cb_ws, ...
                     p_cb_p_dss, ...
                     idx_valids, ...
                     f_p_w2p_p, ...
                     f_dp_p_dh, ...
                     f_p_p2p_p_d, ...
                     f_dp_p_d_dargs, ...
                     idx_update, ...
                     W);
    for it = 1:opts.refine_stereo_params_it_cutoff
        % Store previous params and cost
        params_prev = params;
        cost_prev = cost;

        % Compute delta_params
        delta_params = calc_delta_params(params_prev, ...
                                         p_cb_ws, ...
                                         p_cb_p_dss, ...
                                         idx_valids, ...
                                         f_p_w2p_p, ...
                                         f_dp_p_dh, ...
                                         f_p_p2p_p_d, ...
                                         f_dp_p_d_dargs, ...
                                         idx_update, ...
                                         lambda, ...
                                         W);

        % update params and cost
        params(idx_update) = params_prev(idx_update) + delta_params;
        cost = calc_cost(params, ...
                         p_cb_ws, ...
                         p_cb_p_dss, ...
                         idx_valids, ...
                         f_p_w2p_p, ...
                         f_dp_p_dh, ...
                         f_p_p2p_p_d, ...
                         f_dp_p_d_dargs, ...
                         idx_update, ...
                         W);

        % If cost decreases, decrease lambda and store results; if cost
        % increases, then increase lambda until cost decreases
        if cost < cost_prev
            % Decrease lambda and continue to next iteration
            lambda = lambda/opts.refine_stereo_params_lambda_factor;
        else
            while cost >= cost_prev
                % Increase lambda and recompute params
                lambda = opts.refine_stereo_params_lambda_factor*lambda;

                if lambda >= realmax('single')
                    % This will already be a very, very small step, so just
                    % exit
                    delta_params(:) = 0;
                    cost = cost_prev;
                    params = params_prev;
                    break
                end

                % Compute delta_params
                delta_params = calc_delta_params(params_prev, ...
                                                 p_cb_ws, ...
                                                 p_cb_p_dss, ...
                                                 idx_valids, ...
                                                 f_p_w2p_p, ...
                                                 f_dp_p_dh, ...
                                                 f_p_p2p_p_d, ...
                                                 f_dp_p_d_dargs, ...
                                                 idx_update, ...
                                                 lambda, ...
                                                 W);

                % update params and cost
                params(idx_update) = params_prev(idx_update) + delta_params;
                cost = calc_cost(params, ...
                                 p_cb_ws, ...
                                 p_cb_p_dss, ...
                                 idx_valids, ...
                                 f_p_w2p_p, ...
                                 f_dp_p_dh, ...
                                 f_p_p2p_p_d, ...
                                 f_dp_p_d_dargs, ...
                                 idx_update, ...
                                 W);
            end
        end

        % Exit if change in distance is small
        diff_norm = norm(delta_params);

        % Print iteration stats
        [~, res] = calc_gauss_newton_params(params, ...
                                            p_cb_ws, ...
                                            p_cb_p_dss, ...
                                            idx_valids, ...
                                            f_p_w2p_p, ...
                                            f_dp_p_dh, ...
                                            f_p_p2p_p_d, ...
                                            f_dp_p_d_dargs, ...
                                            idx_update);
        res = reshape(res, 2, [])'; % get in [x y] format
        d_res = sqrt(res(:, 1).^2 + res(:, 2).^2);
        util.verbose_disp(['It #: ' sprintf('% 3u', it) '; ' ...
                           'Median res dist: ' sprintf('% 12.8f', median(d_res)) '; ' ...
                           'MAD res dist: ' sprintf('% 12.8f', 1.4826*median(abs(d_res - median(d_res)))) '; ' ...
                           'Norm of delta_p: ' sprintf('% 12.8f', diff_norm) '; ' ...
                           'Cost: ' sprintf('% 12.8f', cost) '; ' ...
                           'lambda: ' sprintf('% 12.8f', lambda)], ...
                           2, ...
                           opts);

        if diff_norm < opts.refine_stereo_params_norm_cutoff
            break
        end
    end
    if it == opts.refine_stereo_params_it_cutoff
        warning('iterations hit cutoff before converging!!!');
    end

    % Get covariance of parameters
    [jacob, res] = calc_gauss_newton_params(params, ...
                                            p_cb_ws, ...
                                            p_cb_p_dss, ...
                                            idx_valids, ...
                                            f_p_w2p_p, ...
                                            f_dp_p_dh, ...
                                            f_p_p2p_p_d, ...
                                            f_dp_p_d_dargs, ...
                                            true(size(idx_update))); % Mark all as true for final covariance estimation
    [~, ~, ~, cov_params] = alg.safe_lscov(jacob, res, W);
end

function delta_params = calc_delta_params(params, p_ws, p_p_dss, idx_valids, f_p_w2p_p, f_dp_p_dh, f_p_p2p_p_d, f_dp_p_d_dargs, idx_update, lambda, W)
    % Get gauss newton params
    [jacob, res] = calc_gauss_newton_params(params, ...
                                            p_ws, ...
                                            p_p_dss, ...
                                            idx_valids, ...
                                            f_p_w2p_p, ...
                                            f_dp_p_dh, ...
                                            f_p_p2p_p_d, ...
                                            f_dp_p_d_dargs, ...
                                            idx_update);

    % Get gradient
    grad = jacob'*W*res;

    % Get hessian
    hess = jacob'*W*jacob;

    % Add Levenberg–Marquardt damping
    hess = hess + lambda*eye(sum(idx_update));

    % Get change in params
    delta_params = -alg.safe_lscov(hess, grad);
end

function cost = calc_cost(params, p_ws, p_p_dss, idx_valids, f_p_w2p_p, f_dp_p_dh, f_p_p2p_p_d, f_dp_p_d_dargs, idx_update, W)
    % Get residuals
    [~, res] = calc_gauss_newton_params(params, ...
                                        p_ws, ...
                                        p_p_dss, ...
                                        idx_valids, ...
                                        f_p_w2p_p, ...
                                        f_dp_p_dh, ...
                                        f_p_p2p_p_d, ...
                                        f_dp_p_d_dargs, ...
                                        idx_update);

    % Apply weights
    cost = res'*W*res;
end

function [jacob, res] = calc_gauss_newton_params(params, p_ws, p_p_dss, idx_valids, f_p_w2p_p, f_dp_p_dh, f_p_p2p_p_d, f_dp_p_d_dargs, idx_update)
    % Get number of boards
    num_boards = numel(p_p_dss.L);

    % Get number of distortion params
    num_params_d = alg.num_params_d(f_p_p2p_p_d);

    % Handle left camera -------------------------------------------------%
    % Left extrinsics are independent

    % Get intrinsic parameters
    a_L = params(1:3);
    d_L = params(4:3+num_params_d);

    % Get residuals and jacobian
    res_L = zeros(2*sum(vertcat(idx_valids.L{:})), 1);
    jacob_L = sparse(2*sum(vertcat(idx_valids.L{:})), numel(params));
    for i = 1:num_boards
        % Get rotation and translation for left board
        R_L = alg.euler2rot(params(7+2*num_params_d+6*(i-1):9+2*num_params_d+6*(i-1)));
        t_L = params(10+2*num_params_d+6*(i-1):12+2*num_params_d+6*(i-1));

        % Get distorted pixel points
        p_p_d_m_Ls = alg.p_w2p_p_d(p_ws, f_p_w2p_p, R_L, t_L, f_p_p2p_p_d, a_L, d_L);

        % Store residuals - take valid indices into account
        res_L(2*sum(vertcat(idx_valids.L{1:i-1}))+1:2*sum(vertcat(idx_valids.L{1:i}))) = ...
            reshape(vertcat((p_p_d_m_Ls(idx_valids.L{i}, 1)-p_p_dss.L{i}(idx_valids.L{i}, 1))', ...
                            (p_p_d_m_Ls(idx_valids.L{i}, 2)-p_p_dss.L{i}(idx_valids.L{i}, 2))'), [], 1);

        % Intrinsics
        jacob_L(2*sum(vertcat(idx_valids.L{1:i-1}))+1:2*sum(vertcat(idx_valids.L{1:i})), 1:3+num_params_d) = ...
            alg.dp_p_d_dintrinsic(p_ws(idx_valids.L{i}, :), f_p_w2p_p, f_dp_p_dh, R_L, t_L, f_dp_p_d_dargs, a_L, d_L); %#ok<SPRIX>

        % Extrinsics
        dr_L_deuler_L = alg.dr_deuler(alg.rot2euler(R_L));
        drt_L_dm_L = blkdiag(dr_L_deuler_L, eye(3));
        jacob_L(2*sum(vertcat(idx_valids.L{1:i-1}))+1:2*sum(vertcat(idx_valids.L{1:i})), 7+2*num_params_d+6*(i-1):12+2*num_params_d+6*(i-1)) = ...
            alg.dp_p_d_dextrinsic(p_ws(idx_valids.L{i}, :), f_p_w2p_p, f_dp_p_dh, R_L, t_L, f_dp_p_d_dargs, a_L, d_L, drt_L_dm_L); %#ok<SPRIX>
    end

    % Handle right camera ------------------------------------------------%
    % Right extrinsics depend on left extrinsics and left=>right extrinsics

    % Get intrinsic parameters
    a_R = params(4+num_params_d:6+num_params_d);
    d_R = params(7+num_params_d:6+2*num_params_d);

    % Get rotation and translation between left and right board
    R_s = alg.euler2rot(params(7+2*num_params_d+6*num_boards:9+2*num_params_d+6*num_boards));
    t_s = params(10+2*num_params_d+6*num_boards:12+2*num_params_d+6*num_boards);

    % Get residuals and jacobian
    res_R = zeros(2*sum(vertcat(idx_valids.R{:})), 1);
    jacob_R = sparse(2*sum(vertcat(idx_valids.R{:})), numel(params));
    for i = 1:num_boards
        % Get rotation and translation for left board
        R_L = alg.euler2rot(params(7+2*num_params_d+6*(i-1):9+2*num_params_d+6*(i-1)));
        t_L = params(10+2*num_params_d+6*(i-1):12+2*num_params_d+6*(i-1));

        % Get rotation and translation for right board
        R_R = R_s*R_L;
        t_R = R_s*t_L+t_s;

        % Get distorted pixel points
        p_p_d_m_Rs = alg.p_w2p_p_d(p_ws, f_p_w2p_p, R_R, t_R, f_p_p2p_p_d, a_R, d_R);

        % Store residuals - take valid indices into account
        res_R(2*sum(vertcat(idx_valids.R{1:i-1}))+1:2*sum(vertcat(idx_valids.R{1:i}))) = ...
            reshape(vertcat((p_p_d_m_Rs(idx_valids.R{i}, 1)-p_p_dss.R{i}(idx_valids.R{i}, 1))', ...
                            (p_p_d_m_Rs(idx_valids.R{i}, 2)-p_p_dss.R{i}(idx_valids.R{i}, 2))'), [], 1);

        % Intrinsics
        jacob_R(2*sum(vertcat(idx_valids.R{1:i-1}))+1:2*sum(vertcat(idx_valids.R{1:i})), 4+num_params_d:6+2*num_params_d) = ...
            alg.dp_p_d_dintrinsic(p_ws(idx_valids.R{i}, :), f_p_w2p_p, f_dp_p_dh, R_R, t_R, f_dp_p_d_dargs, a_R, d_R); %#ok<SPRIX>

        % Extrinsics - m_L
        drt_R_drt_L = blkdiag(R_s, R_s, R_s, R_s);
        dr_L_deuler_L = alg.dr_deuler(alg.rot2euler(R_L));
        drt_L_dm_L = blkdiag(dr_L_deuler_L, eye(3));
        drt_R_dm_L = drt_R_drt_L*drt_L_dm_L;
        jacob_R(2*sum(vertcat(idx_valids.R{1:i-1}))+1:2*sum(vertcat(idx_valids.R{1:i})), 7+2*num_params_d+6*(i-1):12+2*num_params_d+6*(i-1)) = ...
            alg.dp_p_d_dextrinsic(p_ws(idx_valids.R{i}, :), f_p_w2p_p, f_dp_p_dh, R_R, t_R, f_dp_p_d_dargs, a_R, d_R, drt_R_dm_L); %#ok<SPRIX>

        % Extrinsics - m_s
        drt_R_drt_s = [R_L(1, 1)*eye(3) R_L(2, 1)*eye(3) R_L(3, 1)*eye(3) zeros(3);
                       R_L(1, 2)*eye(3) R_L(2, 2)*eye(3) R_L(3, 2)*eye(3) zeros(3);
                       R_L(1, 3)*eye(3) R_L(2, 3)*eye(3) R_L(3, 3)*eye(3) zeros(3);
                       t_L(1)*eye(3)   t_L(2)*eye(3)   t_L(3)*eye(3)   eye(3)];
        dr_s_deuler_s = alg.dr_deuler(alg.rot2euler(R_s));
        drt_s_dm_s = blkdiag(dr_s_deuler_s, eye(3));
        drt_R_dm_s = drt_R_drt_s*drt_s_dm_s;
        jacob_R(2*sum(vertcat(idx_valids.R{1:i-1}))+1:2*sum(vertcat(idx_valids.R{1:i})), 7+2*num_params_d+6*num_boards:12+2*num_params_d+6*num_boards) = ...
            alg.dp_p_d_dextrinsic(p_ws(idx_valids.R{i}, :), f_p_w2p_p, f_dp_p_dh, R_R, t_R, f_dp_p_d_dargs, a_R, d_R, drt_R_dm_s); %#ok<SPRIX>
    end

    % Concat
    res = vertcat(res_L, res_R);
    jacob = vertcat(jacob_L, jacob_R);

    % Only update specified parameters
    jacob = jacob(:, idx_update);
end
