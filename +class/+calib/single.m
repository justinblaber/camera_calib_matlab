classdef single < class.calib.base
    % This is the class definition for a single calibration.

    methods(Access = public)
        function obj = single(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts)
            obj@class.calib.base(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts);
        end
    end

    methods(Access = private)
        % Params  --------------------------------------------------------%

        function params = get_params(obj, A, d, Rs, ts)
            % params: [a; d; r_1; t_1; ...; r_N; t_N]

            % Intrinsics
            params = vertcat(obj.A2a(A), d);

            % Extrinsics
            for i = 1:numel(Rs)
                params = vertcat(params, ...
                                 obj.R2r(Rs{i}), ...
                                 ts{i}); %#ok<AGROW>
            end
        end

        function [A, d, Rs, ts] = parse_params(obj, params)
            % params: [a; d; r_1; t_1; ...; r_N; t_N]

            % Parse A
            [a, params] = obj.pop_param(params, obj.get_num_params_a());
            A = obj.a2A(a);

            % Parse d
            [d, params] = obj.pop_param(params, obj.get_num_params_d());

            % Parse Rs and ts
            i = 1;
            while numel(params) > 0
                % Parse R
                [r, params] = obj.pop_param(params, obj.get_num_params_r());
                Rs{i} = obj.r2R(r); %#ok<AGROW>

                % Parse t
                [t, params] = obj.pop_param(params, obj.get_num_params_t());
                ts{i} = t; %#ok<AGROW>

                % Increment
                i = i + 1;
            end
        end

        % Residual and jacobian ------------------------------------------%

        function [res, jacob] = calc_res_and_jacob(obj, params, p_cb_ws, p_cb_p_dss, idx_valids)
            % Get number of boards
            num_boards = numel(p_cb_p_dss);

            % Get number of params
            num_params = numel(params);

            % Parse params
            [A, d, Rs, ts] = obj.parse_params(params);

            % Get residuals and jacobian
            num_res = 2*sum(vertcat(idx_valids{:}));
            res = zeros(num_res, 1);
            jacob = sparse(num_res, num_params);
            for i = 1:num_boards
                % Get valid indices for this board
                idx_valid_board = 2*sum(vertcat(idx_valids{1:i-1}))+1:2*sum(vertcat(idx_valids{1:i}));

                % Get valid points
                p_cb_ws_valid = p_cb_ws(idx_valids{i}, :);
                p_cb_p_ds_valid = p_cb_p_dss{i}(idx_valids{i}, :);

                % Get valid model calibration board distorted pixel points
                p_cb_p_d_ms_valid = obj.p_cb_w2p_cb_p_d(p_cb_ws_valid, Rs{i}, ts{i}, A, d);

                % Store residuals
                res(idx_valid_board) = reshape((p_cb_p_d_ms_valid - p_cb_p_ds_valid)', [], 1);

                % Intrinsics
                idx_intrinsic = 1:obj.get_num_params_i();
                jacob(idx_valid_board, idx_intrinsic) = obj.dp_cb_p_d_dintrinsic(p_cb_ws_valid, Rs{i}, ts{i}, A, d); %#ok<SPRIX>

                % Extrinsics
                dR_dr = obj.dR_dr(obj.R2r(Rs{i}));
                dRt_dextrinsic = blkdiag(dR_dr, eye(3));
                idx_extrinsic = obj.get_num_params_i()+(i-1)*obj.get_num_params_e()+1:obj.get_num_params_i()+i*obj.get_num_params_e();
                jacob(idx_valid_board, idx_extrinsic) = obj.dp_cb_p_d_dextrinsic(p_cb_ws_valid, Rs{i}, ts{i}, A, d, dRt_dextrinsic); %#ok<SPRIX>
            end
        end
    end

    methods(Access = public)
        function [A, d, Rs, ts] = refine(obj, A, d, Rs, ts, p_cb_ws, p_cb_p_dss, idx_valids, optimization_type, cov_cb_p_dss)
            % Get opts
            opts = obj.get_opts();

            % Get params
            params = obj.get_params(A, d, Rs, ts);

            % Get number of params
            num_params = numel(params);

            % Determine which parameters to update based on type
            idx_update = false(size(params));
            switch optimization_type
                case 'intrinsic'
                    % Only update camera matrix and distortion params
                    idx_update(1:obj.get_num_params_i()) = true;
                case 'extrinsic'
                    % Only update rotations and translations
                    idx_update(obj.get_num_params_i()+1:num_params) = true;
                case 'full'
                    % Update everything
                    idx_update(1:num_params) = true;
                otherwise
                    error(['Input type of: "' optimization_type '" is not supported']);
            end

            % Get covariance matrix
            if exist('cov_cb_p_dss', 'var')
                cov = vertcat(cov_cb_p_dss{:});                      % Concat
                cov = cov(vertcat(idx_valids{:}));                   % Apply valid indices
                cov = cellfun(@sparse, cov, 'UniformOutput', false); % Make sparse
                cov = blkdiag(cov{:});                               % Create full covariance matrix
            else
                cov = speye(2*sum(vertcat(idx_valids{:})));          % Identity matrix is just simple least squares
            end

            % Get residual and jacobian function
            f_calc_res_and_jacob = @(params)obj.calc_res_and_jacob(params, p_cb_ws, p_cb_p_dss, idx_valids);

            % Levenberg-Marquardt with covariance estimate optimization
            [params, cov_params] = alg.lmcov(f_calc_res_and_jacob, ...
                                             params, ...
                                             cov, ...
                                             idx_update, ...
                                             opts.refine_single_params_lambda_init, ...
                                             opts.refine_single_params_lambda_factor, ...
                                             opts.refine_single_params_it_cutoff, ...
                                             opts.refine_single_params_norm_cutoff, ...
                                             3, ...
                                             opts);

            % Print params
            util.verbose_disp('---', 1, opts);
            util.verbose_disp('Single intrinsic params (+- 3*sigma):', 1, opts);
            % Camera matrix params
            util.verbose_disp('  -Camera: ', 1, opts);
            a_args = obj.get_a_args();
            for i = 1:obj.get_num_params_a()
                obj.print_param(a_args{i}, i, params, cov_params, newline);
            end
            % Distortion params
            util.verbose_disp('  -Distortion: ', 1, opts);
            d_args = obj.get_d_args();
            for i = 1:obj.get_num_params_d()
                obj.print_param(d_args{i}, obj.get_num_params_a()+i, params, cov_params, newline);
            end

            % Parse params
            [A, d, Rs, ts] = obj.parse_params(params);
        end
    end
end
