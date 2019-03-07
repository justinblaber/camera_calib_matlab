classdef stereo < class.calib.base
    % This is the class definition for a stereo calibration.
    
    methods(Access = public)
        function obj = stereo(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts)
            obj@class.calib.base(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts);
        end
    end
    
    methods(Access = private)
        % Params  --------------------------------------------------------%
        
        function params = get_params(obj, A, d, Rs, ts, R_s, t_s)
            % params: [a_L; d_L; a_R; d_R; r_1_L; t_1_L; ...; r_N_L; t_N_L, r_s, t_s]
            
            % Intrinsics
            params = vertcat(obj.A2a(A.L), d.L, obj.A2a(A.R), d.R);
            
            % Extrinsics
            for i = 1:numel(Rs.L)
                params = vertcat(params, ...
                                 obj.R2r(Rs.L{i}), ...
                                 ts.L{i}); %#ok<AGROW>
            end
            params = vertcat(params, ...
                             obj.R2r(R_s), ...
                             t_s);
        end
        
        function [A, d, Rs, ts, R_s, t_s] = parse_params(obj, params)
            % params: [a_L; d_L; a_R; d_R; r_1_L; t_1_L; ...; r_N_L; t_N_L, r_s, t_s]
                                    
            % Parse A.L
            [a.L, params] = obj.pop_param(params, obj.get_num_params_a());
            A.L = obj.a2A(a.L);
                        
            % Parse d.L
            [d.L, params] = obj.pop_param(params, obj.get_num_params_d());
            
            % Parse A.R
            [a.R, params] = obj.pop_param(params, obj.get_num_params_a());
            A.R = obj.a2A(a.R);
            
            % Parse d.R
            [d.R, params] = obj.pop_param(params, obj.get_num_params_d());
            
            % Parse Rs.L and ts.L
            i = 1;
            while numel(params) > obj.get_num_params_e()
                % Parse R
                [r_L, params] = obj.pop_param(params, obj.get_num_params_r());
                Rs.L{i} = obj.r2R(r_L);
                                
                % Parse t
                [t_L, params] = obj.pop_param(params, obj.get_num_params_t());
                ts.L{i} = t_L;
                
                % Increment
                i = i + 1;
            end
            
            % Parse R_s and t_s
            [r_s, params] = obj.pop_param(params, obj.get_num_params_r());
            R_s = obj.r2R(r_s);

            [t_s, params] = obj.pop_param(params, obj.get_num_params_t()); %#ok<ASGLU>

            % Recompute Rs.R and ts.R            
            for i = 1:numel(Rs.L)
                Rs.R{i} = R_s*Rs.L{i};
                ts.R{i} = R_s*ts.L{i} + t_s;
            end
        end
                
        % Residual and jacobian ------------------------------------------%
        
        function [res, jacob] = calc_res_and_jacob(obj, params, p_cb_ws, p_cb_p_dss, idx_valids)
            % Get number of boards
            num_boards = numel(p_cb_p_dss.L);
            
            % Get number of params
            num_params = numel(params);
                        
            % Parse params
            [A, d, Rs, ts, R_s, t_s] = obj.parse_params(params); %#ok<ASGLU>
    
            % Handle left camera -----------------------------------------%
            % Left extrinsics are independent

            % Get residuals and jacobian
            num_res_L = 2*sum(vertcat(idx_valids.L{:}));
            res_L = zeros(num_res_L, 1);
            jacob_L = sparse(num_res_L, num_params);
            for i = 1:num_boards
                % Get valid indices for this board
                idx_valid_board_L = 2*sum(vertcat(idx_valids.L{1:i-1}))+1:2*sum(vertcat(idx_valids.L{1:i}));
                
                % Get valid points
                p_cb_ws_valid_L = p_cb_ws(idx_valids.L{i}, :);
                p_cb_p_ds_valid_L = p_cb_p_dss.L{i}(idx_valids.L{i}, :);
                
                % Get valid model calibration board distorted pixel points
                p_cb_p_d_ms_valid_L = obj.p_cb_w2p_cb_p_d(p_cb_ws_valid_L, Rs.L{i}, ts.L{i}, A.L, d.L);
                
                % Store residuals
                res_L(idx_valid_board_L) = reshape((p_cb_p_d_ms_valid_L - p_cb_p_ds_valid_L)', [], 1);

                % Intrinsics
                idx_intrinsic_L = 1:obj.get_num_params_i();
                jacob_L(idx_valid_board_L, idx_intrinsic_L) = obj.dp_cb_p_d_dintrinsic(p_cb_ws_valid_L, Rs.L{i}, ts.L{i}, A.L, d.L); %#ok<SPRIX>

                % Extrinsics
                dR_L_dr_L = obj.dR_dr(obj.R2r(Rs.L{i}));
                dRt_L_dextrinsic_L = blkdiag(dR_L_dr_L, eye(3));
                idx_extrinsic_L = 2*obj.get_num_params_i()+(i-1)*obj.get_num_params_e()+1:2*obj.get_num_params_i()+i*obj.get_num_params_e();
                jacob_L(idx_valid_board_L, idx_extrinsic_L) = obj.dp_cb_p_d_dextrinsic(p_cb_ws_valid_L, Rs.L{i}, ts.L{i}, A.L, d.L, dRt_L_dextrinsic_L); %#ok<SPRIX>
            end

            % Handle right camera ----------------------------------------%
            % Right extrinsics depend on left extrinsics and left=>right extrinsics

            % Get residuals and jacobian
            num_res_R = 2*sum(vertcat(idx_valids.R{:}));
            res_R = zeros(num_res_R, 1);
            jacob_R = sparse(num_res_R, num_params);
            for i = 1:num_boards
                % Get valid indices for this board
                idx_valid_board_R = 2*sum(vertcat(idx_valids.R{1:i-1}))+1:2*sum(vertcat(idx_valids.R{1:i}));
                
                % Get valid points
                p_cb_ws_valid_R = p_cb_ws(idx_valids.R{i}, :);
                p_cb_p_ds_valid_R = p_cb_p_dss.R{i}(idx_valids.R{i}, :);
                
                % Get valid model calibration board distorted pixel points
                p_cb_p_d_ms_valid_R = obj.p_cb_w2p_cb_p_d(p_cb_ws_valid_R, Rs.R{i}, ts.R{i}, A.R, d.R);

                % Store residuals
                res_R(idx_valid_board_R) = reshape((p_cb_p_d_ms_valid_R - p_cb_p_ds_valid_R)', [], 1);

                % Intrinsics
                idx_intrinsic_R = obj.get_num_params_i()+1:2*obj.get_num_params_i();
                jacob_R(idx_valid_board_R, idx_intrinsic_R) = obj.dp_cb_p_d_dintrinsic(p_cb_ws_valid_R, Rs.R{i}, ts.R{i}, A.R, d.R); %#ok<SPRIX>

                % Extrinsics - L
                dRt_R_dRt_L = blkdiag(R_s, R_s, R_s, R_s);
                dR_L_dr_L = obj.dR_dr(obj.R2r(Rs.L{i}));
                dRt_L_dextrinsic_L = blkdiag(dR_L_dr_L, eye(3));
                dRt_R_dextrinsic_L = dRt_R_dRt_L*dRt_L_dextrinsic_L;
                idx_extrinsic_L = 2*obj.get_num_params_i()+(i-1)*obj.get_num_params_e()+1:2*obj.get_num_params_i()+i*obj.get_num_params_e();
                jacob_R(idx_valid_board_R, idx_extrinsic_L) = obj.dp_cb_p_d_dextrinsic(p_cb_ws_valid_R, Rs.R{i}, ts.R{i}, A.R, d.R, dRt_R_dextrinsic_L); %#ok<SPRIX>

                % Extrinsics - s
                dRt_R_dRt_s = [Rs.L{i}(1, 1)*eye(3), Rs.L{i}(2, 1)*eye(3), Rs.L{i}(3, 1)*eye(3), zeros(3);
                               Rs.L{i}(1, 2)*eye(3), Rs.L{i}(2, 2)*eye(3), Rs.L{i}(3, 2)*eye(3), zeros(3);
                               Rs.L{i}(1, 3)*eye(3), Rs.L{i}(2, 3)*eye(3), Rs.L{i}(3, 3)*eye(3), zeros(3);
                                  ts.L{i}(1)*eye(3),    ts.L{i}(2)*eye(3),    ts.L{i}(3)*eye(3),   eye(3)];
                dR_s_dr_s = obj.dR_dr(obj.R2r(R_s));
                dRt_s_dextrinsic_s = blkdiag(dR_s_dr_s, eye(3));
                dRt_R_dextrinsic_s = dRt_R_dRt_s*dRt_s_dextrinsic_s;
                idx_extrinsic_s = num_params-obj.get_num_params_e()+1:num_params;
                jacob_R(idx_valid_board_R, idx_extrinsic_s) = obj.dp_cb_p_d_dextrinsic(p_cb_ws_valid_R, Rs.R{i}, ts.R{i}, A.R, d.R, dRt_R_dextrinsic_s); %#ok<SPRIX>
            end

            % Concat
            res = vertcat(res_L, res_R);
            jacob = vertcat(jacob_L, jacob_R);
        end    
    end    
    
    methods(Access = public)                
        function [A, d, Rs, ts] = refine(obj, A, d, Rs, ts, R_s, t_s, p_cb_ws, p_cb_p_dss, idx_valids, optimization_type, cov_cb_p_dss)
            % Get opts
            opts = obj.get_opts();
                                   
            % Get params
            params = obj.get_params(A, d, Rs, ts, R_s, t_s);           
                                    
            % Get number of params
            num_params = numel(params);
            
            % Determine which parameters to update based on type
            idx_update = false(size(params));
            switch optimization_type
                case 'extrinsic'
                    % Only update rotations and translations
                    idx_update(2*obj.get_num_params_i()+1:num_params) = true;
                case 'full'
                    % Update everything
                    idx_update(1:num_params) = true;
                otherwise
                    error(['Input type of: "' optimization_type '" is not supported']);
            end

            % Get covariance matrix
            if exist('cov_cb_p_dss', 'var')
                cov = vertcat(cov_cb_p_dss.L{:}, cov_cb_p_dss.R{:});           % Concat
                cov = cov(vertcat(idx_valids.L{:}, idx_valids.R{:}));          % Apply valid indices
                cov = cellfun(@sparse, cov, 'UniformOutput', false);           % Make sparse
                cov = blkdiag(cov{:});                                         % Create full covariance matrix
            else
                cov = speye(2*sum(vertcat(idx_valids.L{:}, idx_valids.R{:}))); % Identity matrix is just simple least squares
            end
    
            % Get residual and jacobian function
            f_calc_res_and_jacob = @(params)obj.calc_res_and_jacob(params, p_cb_ws, p_cb_p_dss, idx_valids);
            
            % Levenberg-Marquardt with covariance estimate optimization
            [params, cov_params] = alg.lmcov(f_calc_res_and_jacob, ...
                                             params, ...
                                             cov, ...
                                             idx_update, ...
                                             opts.refine_stereo_params_lambda_init, ...
                                             opts.refine_stereo_params_lambda_factor, ...
                                             opts.refine_stereo_params_it_cutoff, ...
                                             opts.refine_stereo_params_norm_cutoff, ...
                                             2, ...
                                             opts);         
                                          
            % Print params
            util.verbose_disp('------', 1, opts);
            util.verbose_disp('Stereo intrinsic params (+- 3*sigma):', 1, opts);
            % Camera matrix params
            util.verbose_disp('  -Camera (L):                         -Camera (R): ', 1, opts);
            a_args = obj.get_a_args();
            for i = 1:obj.get_num_params_a()
                obj.print_param(a_args{i}, i,                        params, cov_params,    '  ');
                obj.print_param(a_args{i}, obj.get_num_params_i()+i, params, cov_params, newline);
            end            
            % Distortion params
            util.verbose_disp('  -Distortion (L):                     -Distortion (R): ', 1, opts);
            d_args = obj.get_d_args();
            for i = 1:obj.get_num_params_d()
                obj.print_param(d_args{i}, obj.get_num_params_a()+i,                        params, cov_params,    '  ');
                obj.print_param(d_args{i}, obj.get_num_params_i()+obj.get_num_params_a()+i, params, cov_params, newline);
            end
            % Stereo params
            util.verbose_disp('Stereo extrinsic params (+- 3*sigma):', 1, opts);
            % Rotation
            util.verbose_disp('  -Rotation (L => R):', 1, opts);
            r_args = obj.get_r_args();
            for i = 1:obj.get_num_params_r()
                obj.print_param(r_args{i}, num_params-obj.get_num_params_e()+i, params, cov_params, newline);
            end
            % Translation
            util.verbose_disp('  -Translation (L => R):', 1, opts);
            t_args = {'x', 'y', 'z'};            
            for i = 1:obj.get_num_params_t()
                obj.print_param(t_args{i}, num_params-obj.get_num_params_t()+i, params, cov_params, newline);
            end

            % Parse params
            [A, d, Rs, ts] = obj.parse_params(params);
        end
    end
end
