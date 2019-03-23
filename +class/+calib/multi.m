classdef multi < class.calib.base
    % This is the class definition for a multi calibration.

    methods(Access = public)
        function obj = multi(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts)
            obj@class.calib.base(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts);
        end
    end

    methods(Access = private)
        % Params  --------------------------------------------------------%

        function params = get_params(obj, As, ds, Rss, tss, R_1s, t_1s)
            % params: [a_1; d_1; a_2; d_2; ... a_M; d_M; ...
            %          r_1; t_1; r_2; t_2; ... r_N; t_N; ...
            %          r_1_1; t_1_1; r_1_2; t_1_2; ...; r_1_M, t_1_M]

            % Get number of cameras and number of calibration boards
            num_cams = numel(As);
            num_boards = numel(Rss{1});
            
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
                                 obj.R2r(Rss{1}{i}), ...
                                 tss{1}{i}); %#ok<AGROW>
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
                Rs{i} = obj.r2R(r); %#ok<AGROW>

                % Parse t
                [ts{i}, params] = obj.pop_param(params, obj.get_num_params_t()); %#ok<AGROW>
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

        function [res, jacob] = calc_res_and_jacob(obj, params, p_cb_ws, p_cb_p_dsss, idx_validss)
            % Get number of cameras and number of calibration boards
            num_cams = numel(p_cb_p_dsss);
            num_boards = numel(p_cb_p_dsss{1});
            
            % Get number of params
            num_params = numel(params);

            % Parse params
            [As, ds, Rs, ts, R_1s, t_1s] = obj.parse_params(params, num_cams, num_boards);

            % Loop over cameras and build residuals and jacobian ---------%
            res = [];
            jacob = sparse([]);
            
            for i = 1:num_cams
                % Get residuals and jacobian for this camera
                num_res_i = 2*sum(vertcat(idx_validss{i}{:}));
                res_i = zeros(num_res_i, 1);
                jacob_i = sparse(num_res_i, num_params);
                                
                % Cycle over boards
                for j = 1:num_boards                    
                    % Get rotation and translation for this board                    
                    R_board = R_1s{i}*Rs{j};
                    t_board = R_1s{i}*ts{j} + t_1s{i};
                    
                    % Get valid indices for this board
                    idx_valid_board = 2*sum(vertcat(idx_validss{i}{1:j-1}))+1:2*sum(vertcat(idx_validss{i}{1:j}));

                    % Get valid points
                    p_cb_ws_valid = p_cb_ws(idx_validss{i}{j}, :);
                    p_cb_p_ds_valid = p_cb_p_dsss{i}{j}(idx_validss{i}{j}, :);

                    % Get valid model calibration board distorted pixel points
                    p_cb_p_d_ms_valid = obj.p_cb_w2p_cb_p_d(p_cb_ws_valid, R_board, t_board, As{i}, ds{i});

                    % Store residuals
                    res_i(idx_valid_board) = reshape((p_cb_p_d_ms_valid - p_cb_p_ds_valid)', [], 1);

                    % Intrinsics
                    idx_intrinsic_i = (i-1)*obj.get_num_params_i()+1:i*obj.get_num_params_i();
                    jacob_i(idx_valid_board, idx_intrinsic_i) = obj.dp_cb_p_d_dintrinsic(p_cb_ws_valid, R_board, t_board, As{i}, ds{i}); %#ok<SPRIX>

                    % Extrinsics - "1"
                    dRt_i_dRt_1 = blkdiag(R_1s{i}, R_1s{i}, R_1s{i}, R_1s{i});
                    dR_1_dr_1 = obj.dR_dr(obj.R2r(Rs{j}));
                    dRt_1_dextrinsic_1 = blkdiag(dR_1_dr_1, eye(3));
                    dRt_i_dextrinsic_1 = dRt_i_dRt_1*dRt_1_dextrinsic_1;
                    idx_extrinsic_1 = num_cams*obj.get_num_params_i()+(j-1)*obj.get_num_params_e()+1:num_cams*obj.get_num_params_i()+j*obj.get_num_params_e();
                    jacob_i(idx_valid_board, idx_extrinsic_1) = obj.dp_cb_p_d_dextrinsic(p_cb_ws_valid, R_board, t_board, As{i}, ds{i}, dRt_i_dextrinsic_1); %#ok<SPRIX>

                    % Extrinsics - "1_i"
                    dRt_i_dRt_1_i = [Rs{j}(1, 1)*eye(3), Rs{j}(2, 1)*eye(3), Rs{j}(3, 1)*eye(3), zeros(3);
                                     Rs{j}(1, 2)*eye(3), Rs{j}(2, 2)*eye(3), Rs{j}(3, 2)*eye(3), zeros(3);
                                     Rs{j}(1, 3)*eye(3), Rs{j}(2, 3)*eye(3), Rs{j}(3, 3)*eye(3), zeros(3);
                                        ts{j}(1)*eye(3),    ts{j}(2)*eye(3),    ts{j}(3)*eye(3),   eye(3)];
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
    end

    methods(Access = public)
        function [As, ds, Rss, tss, R_1s, t_1s] = refine(obj, As, ds, Rss, tss, R_1s, t_1s, p_cb_ws, p_cb_p_dsss, idx_validss, optimization_type, cov_cb_p_dsss)
            % Get opts
            opts = obj.get_opts();

            % Get number of cameras and number of calibration boards
            num_cams = numel(As);
            num_boards = numel(Rss{1});
            
            % Get params
            params = obj.get_params(As, ds, Rss, tss, R_1s, t_1s);

            % Get number of params
            num_params = numel(params);

            % Determine which parameters to update based on type
            idx_update = false(size(params));
            switch optimization_type
                case 'extrinsic'
                    % Only update rotations and translations
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
                % Get covariance matrix
                cov = [cov_cb_p_dsss{:}];                                  
                cov = vertcat(cov{:});
                
                % Apply valid indices
                idx = [idx_validss{:}];
                idx = vertcat(idx{:});                
                cov = cov(idx);
                
                % Make sparse
                cov = cellfun(@sparse, cov, 'UniformOutput', false);
                
                % Create sparse matrix
                cov = blkdiag(cov{:});
            else
                % Create sparse identity matrix which is just least squares
                cov = speye(2*sum(cellfun(@sum, [idx_validss{:}])));
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
            util.verbose_disp('Multi intrinsic params (+- 3*sigma):', 1, opts);
            
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
                    Rss{i}{j} = R_1s{i}*Rs{j};
                    tss{i}{j} = R_1s{i}*ts{j} + t_1s{i};
                end
            end
        end
    end
end
