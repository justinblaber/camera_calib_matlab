classdef base < class.calib.A_intf & class.calib.R_intf & class.calib.cb_w2p_intf & class.distortion.intf %#ok<*PROPLC>
    % This is the base class definition for a calibration.

    properties(Access = private)
        obj_A
        obj_R
        obj_cb_w2p
        obj_distortion
        opts
    end
    
    methods(Access = private)        
        function opts = get_opts(obj)
            opts = obj.opts;
        end
    end

    methods(Access = public)
        function obj = base(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts)
            obj.obj_A = obj_A;
            obj.obj_R = obj_R;
            obj.obj_cb_w2p = obj_cb_w2p;
            obj.obj_distortion = obj_distortion;
            obj.opts = opts;
        end

        % Forward A interface --------------------------------------------%

        function A = a2A(obj, a)
            A = obj.obj_A.a2A(a);
        end

        function a = A2a(obj, A)
            a = obj.obj_A.A2a(A);
        end

        function dA_da = dA_da(obj, a)
            dA_da = obj.obj_A.dA_da(a);
        end

        function num = get_num_params_a(obj)
            num = obj.obj_A.get_num_params_a();
        end

        function args = get_a_args(obj)
            args = obj.obj_A.get_a_args();
        end

        % Forward R interface --------------------------------------------%

        function R = r2R(obj, r)
            R = obj.obj_R.r2R(r);
        end

        function r = R2r(obj, R)
            r = obj.obj_R.R2r(R);
        end

        function dR_dr = dR_dr(obj, r)
            dR_dr = obj.obj_R.dR_dr(r);
        end

        function num = get_num_params_r(obj)
            num = obj.obj_R.get_num_params_r();
        end

        function args = get_r_args(obj)
            args = obj.obj_R.get_r_args();
        end

        % Forward cb_w2p interface ---------------------------------------%

        function p_cb_ps = p_cb_w2p_cb_p(obj, p_cb_ws, H)
            p_cb_ps = obj.obj_cb_w2p.p_cb_w2p_cb_p(p_cb_ws, H);
        end

        function jacob = dp_cb_p_dH(obj, p_cb_ws, H)
            jacob = obj.obj_cb_w2p.dp_cb_p_dH(p_cb_ws, H);
        end

        function H_w2p = homography_cb_w2p(obj, p_cb_ws, p_cb_ps, cov)
            if ~exist('cov', 'var')
                H_w2p = obj.obj_cb_w2p.homography_cb_w2p(p_cb_ws, p_cb_ps);
            else
                H_w2p = obj.obj_cb_w2p.homography_cb_w2p(p_cb_ws, p_cb_ps, cov);
            end
        end

        % Forward distortion interface -----------------------------------%

        function num_params_d = get_num_params_d(obj)
            num_params_d = obj.obj_distortion.get_num_params_d();
        end

        function args = get_d_args(obj)
            args = obj.obj_distortion.get_d_args();
        end

        function jacob = dp_p_d_dp_p(obj, p_ps, A, d)
            jacob = obj.obj_distortion.dp_p_d_dp_p(p_ps, A, d);
        end

        function jacob = dp_p_d_dA(obj, p_ps, A, d)
            jacob = obj.obj_distortion.dp_p_d_dA(p_ps, A, d);
        end

        function jacob = dp_p_d_dd(obj, p_ps, A, d)
            jacob = obj.obj_distortion.dp_p_d_dd(p_ps, A, d);
        end

        function p_p_ds = p_p2p_p_d(obj, p_ps, A, d)
            p_p_ds = obj.obj_distortion.p_p2p_p_d(p_ps, A, d);
        end

        function p_ps = p_p_d2p_p(obj, p_p_ds, p_ps_init, A, d)
            p_ps = obj.obj_distortion.p_p_d2p_p(p_p_ds, p_ps_init, A, d);
        end

        % Homography stuff -----------------------------------------------%

        function H = ARt2H(obj, A, R, t) %#ok<INUSL>
            H = A*[R(:, 1) R(:, 2) t];
        end

        function jacob = dH_dA(obj, A, R, t) %#ok<INUSL>
            jacob = [R(1, 1)*eye(3), R(2, 1)*eye(3), R(3, 1)*eye(3); ...
                     R(1, 2)*eye(3), R(2, 2)*eye(3), R(3, 2)*eye(3); ...
                        t(1)*eye(3),    t(2)*eye(3),    t(3)*eye(3)];
        end

        function jacob = dH_dR(obj, A, R, t) %#ok<INUSD,INUSL>
            jacob = [       A, zeros(3), zeros(3);
                     zeros(3),        A, zeros(3);
                     zeros(3), zeros(3), zeros(3)];
        end

        function jacob = dH_dt(obj, A, R, t) %#ok<INUSD,INUSL>
            jacob = [zeros(3);
                     zeros(3);
                           A];
        end

        % Distorted calibration board pixel points -----------------------%

        function p_cb_p_d_ms = p_cb_w2p_cb_p_d(obj, p_cb_ws, R, t, A, d)
            % Get calibration board pixel points
            p_cb_ps = obj.p_cb_w2p_cb_p(p_cb_ws, obj.ARt2H(A, R, t));

            % Get model calibration board distorted pixel points
            p_cb_p_d_ms = obj.p_p2p_p_d(p_cb_ps, A, d);
        end

        function jacob = dp_cb_p_d_dintrinsic(obj, p_cb_ws, R, t, A, d)
            % Get homography
            H = obj.ARt2H(A, R, t);

            % Get calibration board pixel points
            p_cb_ps = obj.p_cb_w2p_cb_p(p_cb_ws, H);

            % Get dp_cb_p_d/dH
            dp_cb_p_dH = obj.dp_cb_p_dH(p_cb_ws, H);
            dp_cb_p_d_dp_cb_p = obj.dp_p_d_dp_p(p_cb_ps, A, d);
            dp_cb_p_d_dH = dp_cb_p_d_dp_cb_p*dp_cb_p_dH;

            % 1) a: [dp_cb_p_d/dH]*[dH/dA]*[dA/da] + [dp_cb_p_d/dA]*[dA/da]
            dH_dA = obj.dH_dA(A, R, t);
            dA_da = obj.dA_da(obj.A2a(A));
            dp_cb_p_d_dA = obj.dp_p_d_dA(p_cb_ps, A, d);
            dp_cb_p_d_da = dp_cb_p_d_dH*dH_dA*dA_da + dp_cb_p_d_dA*dA_da;

            % 2) d: [dp_cb_p_d/dd] - assumes p_cb_p has no dependence on these parameters
            dp_cb_p_d_dd = obj.dp_p_d_dd(p_cb_ps, A, d);

            % Form jacobian
            jacob = [dp_cb_p_d_da dp_cb_p_d_dd];
        end

        function jacob = dp_cb_p_d_dextrinsic(obj, p_cb_ws, R, t, A, d, dRt_dextrinsic)
            % Get homography
            H = obj.ARt2H(A, R, t);

            % Get calibration board pixel points
            p_cb_ps = obj.p_cb_w2p_cb_p(p_cb_ws, H);

            % Get dp_cb_p_d/dH
            dp_cb_p_dH = obj.dp_cb_p_dH(p_cb_ws, H);
            dp_cb_p_d_dp_cb_p = obj.dp_p_d_dp_p(p_cb_ps, A, d);
            dp_cb_p_d_dH = dp_cb_p_d_dp_cb_p*dp_cb_p_dH;

            % Get dH/dRt
            dH_dRt = [obj.dH_dR(A, R, t) obj.dH_dt(A, R, t)];

            % Form jacobian
            jacob = dp_cb_p_d_dH*dH_dRt*dRt_dextrinsic;
        end

        % Params stuff ---------------------------------------------------%

        function num_params_t = get_num_params_t(obj) %#ok<MANU>
            num_params_t = 3;
        end

        function num_params_i = get_num_params_i(obj)
            num_params_i = obj.get_num_params_a() + obj.get_num_params_d();
        end

        function num_params_e = get_num_params_e(obj)
            num_params_e = obj.get_num_params_r() + obj.get_num_params_t();
        end

        function [param, params] = pop_param(obj, params, num_param) %#ok<INUSL>
            param = params(1:num_param);
            params(1:num_param) = [];
        end

        function print_param(obj, s_param, idx, params, cov_params, suffix)
            util.verbose_fprintf([pad(['    ' s_param ': '], 13) sprintf('% 10.4f', params(idx)) ' +- ' sprintf('% 8.4f', 3*sqrt(cov_params(idx, idx))) suffix], 1, obj.get_opts());
        end
        
        function params = get_params(obj, As, ds, Rs, ts, R_1s, t_1s)
            % params: [a_1; d_1; a_2; d_2; ... a_M; d_M; ...
            %          r_1; t_1; r_2; t_2; ... r_N; t_N; ...
            %          r_1_1; t_1_1; r_1_2; t_1_2; ...; r_1_M, t_1_M]
                        
            % Get number of cameras and boards
            num_cams = size(Rs, 2);
            num_boards = size(Rs, 1);
            
            % Initialize params
            params = [];
            
            % Intrinsics
            for i = 1:num_cams
                params = vertcat(params, ...
                                 obj.A2a(As{i}), ...
                                 ds{i}); %#ok<AGROW>
            end
                
            % Camera 1 extrinsics
            for i = 1:num_boards
                params = vertcat(params, ...
                                 obj.R2r(Rs{i, 1}), ...
                                 ts{i, 1}); %#ok<AGROW>
            end
            
            % Camera 1 to camera i extrinsics
            for i = 1:num_cams
                params = vertcat(params, ...
                                 obj.R2r(R_1s{i}), ...
                                 t_1s{i}); %#ok<AGROW>
            end
        end

        function [As, ds, Rs, ts, R_1s, t_1s] = parse_params(obj, params, num_cams, num_boards)
            % params: [a_1; d_1; a_2; d_2; ... a_M; d_M; ...
            %          r_1; t_1; r_2; t_2; ... r_N; t_N; ...
            %          r_1_1; t_1_1; r_1_2; t_1_2; ...; r_1_M, t_1_M]

            % Intrinsics
            for i = 1:num_cams
                % Parse A
                [a, params] = obj.pop_param(params, obj.get_num_params_a());
                As{i} = obj.a2A(a); %#ok<AGROW>
                
                % Parse d
                [ds{i}, params] = obj.pop_param(params, obj.get_num_params_d()); %#ok<AGROW>
            end
            
            % Camera 1 extrinsics
            for i = 1:num_boards
                % Parse R
                [r, params] = obj.pop_param(params, obj.get_num_params_r());
                Rs{i, 1} = obj.r2R(r); %#ok<AGROW>

                % Parse t
                [ts{i, 1}, params] = obj.pop_param(params, obj.get_num_params_t()); %#ok<AGROW>
            end
                        
            % Camera 1 to camera i extrinsics
            for i = 1:num_cams
                % Parse R
                [r, params] = obj.pop_param(params, obj.get_num_params_r());
                R_1s{i} = obj.r2R(r); %#ok<AGROW>

                % Parse t
                [t_1s{i}, params] = obj.pop_param(params, obj.get_num_params_t()); %#ok<AGROW>
            end
        end

        % Residual and jacobian ------------------------------------------%

        function [res, jacob] = calc_res_and_jacob(obj, params, p_cb_ws, p_cb_p_dss, idx_validss)            
            % Get number of params
            num_params = numel(params);

            % Get number of cameras and boards
            num_cams = size(p_cb_p_dss, 2);
            num_boards = size(p_cb_p_dss, 1);
            
            % Parse params
            [As, ds, Rs, ts, R_1s, t_1s] = obj.parse_params(params, num_cams, num_boards);

            % Build residuals and jacobian -------------------------------%
            
            res = [];
            jacob = sparse([]);
            for i = 1:num_cams % Iterate over cameras
                % Get residuals and jacobian for this camera
                num_res_i = 2*sum(vertcat(idx_validss{:, i}));
                res_i = zeros(num_res_i, 1);
                jacob_i = sparse(num_res_i, num_params);
                                                
                for j = 1:num_boards % Iterate over boards        
                    % Get rotation and translation for this board                    
                    R_board = R_1s{i}*Rs{j, 1};
                    t_board = R_1s{i}*ts{j, 1} + t_1s{i};
                    
                    % Get valid indices for this board
                    idx_valid_board = 2*sum(vertcat(idx_validss{1:j-1, i}))+1:2*sum(vertcat(idx_validss{1:j, i}));

                    % Get valid points
                    p_cb_ws_valid = p_cb_ws(idx_validss{j, i}, :);
                    p_cb_p_ds_valid = p_cb_p_dss{j, i}(idx_validss{j, i}, :);

                    % Get valid model calibration board distorted pixel points
                    p_cb_p_d_ms_valid = obj.p_cb_w2p_cb_p_d(p_cb_ws_valid, R_board, t_board, As{i}, ds{i});

                    % Store residuals
                    res_i(idx_valid_board) = reshape((p_cb_p_d_ms_valid - p_cb_p_ds_valid)', [], 1);

                    % Intrinsics
                    idx_intrinsic_i = (i-1)*obj.get_num_params_i()+1:i*obj.get_num_params_i();
                    jacob_i(idx_valid_board, idx_intrinsic_i) = obj.dp_cb_p_d_dintrinsic(p_cb_ws_valid, R_board, t_board, As{i}, ds{i}); %#ok<SPRIX>

                    % Extrinsics - "1"
                    dRt_i_dRt_1 = blkdiag(R_1s{i}, R_1s{i}, R_1s{i}, R_1s{i});
                    dR_1_dr_1 = obj.dR_dr(obj.R2r(Rs{j, 1}));
                    dRt_1_dextrinsic_1 = blkdiag(dR_1_dr_1, eye(3));
                    dRt_i_dextrinsic_1 = dRt_i_dRt_1*dRt_1_dextrinsic_1;
                    idx_extrinsic_1 = num_cams*obj.get_num_params_i()+(j-1)*obj.get_num_params_e()+1:num_cams*obj.get_num_params_i()+j*obj.get_num_params_e();
                    jacob_i(idx_valid_board, idx_extrinsic_1) = obj.dp_cb_p_d_dextrinsic(p_cb_ws_valid, R_board, t_board, As{i}, ds{i}, dRt_i_dextrinsic_1); %#ok<SPRIX>

                    % Extrinsics - "1_i"
                    dRt_i_dRt_1_i = [Rs{j, 1}(1, 1)*eye(3), Rs{j, 1}(2, 1)*eye(3), Rs{j, 1}(3, 1)*eye(3), zeros(3);
                                     Rs{j, 1}(1, 2)*eye(3), Rs{j, 1}(2, 2)*eye(3), Rs{j, 1}(3, 2)*eye(3), zeros(3);
                                     Rs{j, 1}(1, 3)*eye(3), Rs{j, 1}(2, 3)*eye(3), Rs{j, 1}(3, 3)*eye(3), zeros(3);
                                        ts{j, 1}(1)*eye(3),    ts{j, 1}(2)*eye(3),    ts{j, 1}(3)*eye(3),   eye(3)];
                    dR_1_i_dr_1_i = obj.dR_dr(obj.R2r(R_1s{i}));
                    dRt_1_i_dextrinsic_1_i = blkdiag(dR_1_i_dr_1_i, eye(3));
                    dRt_i_dextrinsic_1_i = dRt_i_dRt_1_i*dRt_1_i_dextrinsic_1_i;
                    idx_extrinsic_1_i = num_cams*obj.get_num_params_i()+num_boards*obj.get_num_params_e()+(i-1)*obj.get_num_params_e()+1:num_cams*obj.get_num_params_i()+num_boards*obj.get_num_params_e()+i*obj.get_num_params_e();
                    jacob_i(idx_valid_board, idx_extrinsic_1_i) = obj.dp_cb_p_d_dextrinsic(p_cb_ws_valid, R_board, t_board, As{i}, ds{i}, dRt_i_dextrinsic_1_i); %#ok<SPRIX>
                end

                % Concat
                res = vertcat(res, res_i); %#ok<AGROW>
                jacob = vertcat(jacob, jacob_i); %#ok<AGROW>
            end
        end
        
        % Refine ---------------------------------------------------------%
        
        function [As, ds, Rs, ts, R_1s, t_1s] = refine(obj, As, ds, Rs, ts, R_1s, t_1s, p_cb_ws, p_cb_p_dsss, idx_validss, optimization_type, cov_cb_p_dss)
            % Get opts
            opts = obj.get_opts(); 

            % Get number of cameras and boards
            num_cams = size(Rs, 2);
            num_boards = size(Rs, 1);
            
            % Get params
            params = obj.get_params(As, ds, Rs, ts, R_1s, t_1s);

            % Get number of params
            num_params = numel(params);

            % Determine which parameters to update based on type
            idx_update = false(size(params));
            switch optimization_type
                case 'intrinsic'
                    % Only update intrinsics
                    idx_update(1:num_cams*obj.get_num_params_i()) = true;
                case 'extrinsic'
                    % Only update extrinsics
                    idx_update(num_cams*obj.get_num_params_i()+1:num_params) = true;
                case 'full'
                    % Update everything
                    idx_update(1:num_params) = true;
                otherwise
                    error(['Input type of: "' optimization_type '" is not supported']);
            end
            
            % Disable r_1_1 and t_1_1
            idx_update(num_cams*obj.get_num_params_i()+num_boards*obj.get_num_params_e()+1: ...
                       num_cams*obj.get_num_params_i()+(num_boards+1)*obj.get_num_params_e()) = false;

            % Get covariance matrix
            if exist('cov_cb_p_dss', 'var')                
                cov = vertcat(cov_cb_p_dss{:});                      % Append covarianced
                cov = cov(vertcat(idx_validss{:}));                  % Apply valid indices
                cov = cellfun(@sparse, cov, 'UniformOutput', false); % Make sparse
                cov = blkdiag(cov{:});                               % Create sparse matrix
            else                
                cov = speye(2*sum(vertcat(idx_validss{:}))); % Sparse identity matrix is just least squares
            end

            % Get residual and jacobian function
            f_calc_res_and_jacob = @(params)obj.calc_res_and_jacob(params, p_cb_ws, p_cb_p_dsss, idx_validss);

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
            util.verbose_disp('Intrinsic params (+- 3*sigma):', 1, opts);
            
            % Camera matrix params
            util.verbose_fprintf('  ', 1, opts);
            for i = 1:num_cams
                util.verbose_fprintf(['-Camera (' num2str(i) '):                         '], 1, opts);
            end
            util.verbose_fprintf(newline, 1, opts);            
            a_args = obj.get_a_args();
            for i = 1:obj.get_num_params_a()
                for j = 1:num_cams
                    obj.print_param(a_args{i}, (j-1)*obj.get_num_params_i()+i, params, cov_params, '  ');
                end
                util.verbose_fprintf(newline, 1, opts); 
            end 
            
            % Distortion params            
            util.verbose_fprintf('  ', 1, opts);
            for i = 1:num_cams
                util.verbose_fprintf(['-Distortion (' num2str(i) '):                     '], 1, opts);
            end
            util.verbose_fprintf(newline, 1, opts);
            d_args = obj.get_d_args();
            for i = 1:obj.get_num_params_d()
                for j = 1:num_cams
                    obj.print_param(d_args{i}, (j-1)*obj.get_num_params_i()+obj.get_num_params_a()+i, params, cov_params, '  ');
                end
                util.verbose_fprintf(newline, 1, opts); 
            end
                
            % Relative extrinsic params
            util.verbose_disp('Relative extrinsic params (+- 3*sigma):', 1, opts);
            
            % Relative rotation       
            util.verbose_fprintf('  ', 1, opts);
            for i = 1:num_cams
                util.verbose_fprintf(['-Rotation (' num2str(1) ' => ' num2str(i) '):                  '], 1, opts);
            end
            util.verbose_fprintf(newline, 1, opts);
            r_args = obj.get_r_args();
            for i = 1:obj.get_num_params_r()
                for j = 1:num_cams
                    obj.print_param(r_args{i}, num_cams*obj.get_num_params_i()+(num_boards+j-1)*obj.get_num_params_e()+i, params, cov_params, '  ');
                end
                util.verbose_fprintf(newline, 1, opts); 
            end
            
            % Relative translation     
            util.verbose_fprintf('  ', 1, opts);
            for i = 1:num_cams
                util.verbose_fprintf(['-Translation (' num2str(1) ' => ' num2str(i) '):               '], 1, opts);
            end
            util.verbose_fprintf(newline, 1, opts);
            t_args = {'x', 'y', 'z'};
            for i = 1:obj.get_num_params_t()
                for j = 1:num_cams
                    obj.print_param(t_args{i}, num_cams*obj.get_num_params_i()+(num_boards+j-1)*obj.get_num_params_e()+obj.get_num_params_r()+i, params, cov_params, '  ');
                end
                util.verbose_fprintf(newline, 1, opts); 
            end

            % Parse params
            [As, ds, Rs, ts, R_1s, t_1s] = obj.parse_params(params, num_cams, num_boards);
            
            % Set outputs
            for i = 1:num_cams
                for j = 1:num_boards
                    Rs{j, i} = R_1s{i}*Rs{j, 1};
                    ts{j, i} = R_1s{i}*ts{j, 1} + t_1s{i};
                end
            end
        end
    end
end
