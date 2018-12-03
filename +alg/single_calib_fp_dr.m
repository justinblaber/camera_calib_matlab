function calib = single_calib_fp_dr(img_cbs,p_fp_p_dss,calib_config,intrin)
    % Performs camera calibration using "four point distortion refinement"
    % method.
    % 
    % Inputs:
    %   img_cbs - util.img; Nx1 calibration board images
    %   p_fp_p_dss - cell; Nx1 cell of four point boxes around the
    %       calibration board images in distorted pixel coordinates.
    %   calib_config - struct; struct returned by util.read_calib_config()
    %   intrin - struct; optional. If passed in, intrinsics will not be
    %       optimized.
    %       .A - array; 3x3 camera matrix
    %       .d - array; Mx1 array of distortion coefficients
    % 
    % Outputs:
    %   calib - struct;
    %       .config - struct; copy of input calib_config
    %       .intrin - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %       .extrin - struct; Nx1 struct containing extrinsics
    %           .img_path - string; path to calibration board image
    %           .R - array; 3x3 rotation matrix
    %           .t - array; 3x1 translation vector
    %           .p_fp_p_ds - array; four point box around the
    %               calibration board image in distorted pixel coordinates
    %           .p_cb_p_ds - array; calibration board points in distorted
    %               pixel coordinates
    %           .cov_cb_p_ds - cell; covariances of calibration board 
    %               points in distorted pixel coordinates
    %           .idx_valid - array; valid calibration board points
    %           .debug - cell; debugging info
    %       .debug; struct;
    %           .params - array; raw parameter vector returned by 
    %               alg.refine_single_params()
    %           .cov_params - array; covariance of parameters returned by 
    %               alg.refine_single_params()
    
    util.verbose_disp('------',1,calib_config);
    util.verbose_disp('Performing single calibration with four point distortion refinement method...',1,calib_config);
        
    % Get distortion function
    sym_p_p2p_p_d = eval(calib_config.sym_p_p2p_p_d);
    
    % If intrinsics are passed in, don't optimize for them
    num_params_d = alg.num_params_d(sym_p_p2p_p_d);
    if exist('intrin','var')
        % Validate A - assumes single focal length and no skew
        if intrin.A(1,1) ~= intrin.A(2,2) || any(intrin.A([2 3 4 6]))         
            error('Invalid initial camera matrix!')
        end
            
        % Validate d - make sure length of d is correct
        if size(intrin.d,1) ~= num_params_d || size(intrin.d,2) ~= 1  
            error('Invalid initial distortion vector!')
        end
        
        a = alg.A2a(intrin.A);
        d = intrin.d;
        optimization_type = 'extrinsic';
        distortion_refinement_it_cutoff = 1;
    else
        optimization_type = 'full';
        if num_params_d > 0
            distortion_refinement_it_cutoff = calib_config.distortion_refinement_it_cutoff;
        else
            distortion_refinement_it_cutoff = 1; % Model has no distortion parameters
        end
    end
    
    % Get the calibration board points and the four point box on the 
    % calibration board in world coordinates.
    [p_cb_ws, p_fp_ws] = alg.p_cb_w(calib_config);
                
    % Get number of boards
    num_boards = numel(img_cbs);
    
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
        
    % Perform distortion refinement iterations, which iteratively applies
    % distortion correction to images to help refine target point locations
    for it = 1:distortion_refinement_it_cutoff
        util.verbose_disp('---',1,calib_config);
        util.verbose_disp(['Performing distortion refinement iteration: ' num2str(it) '...'],1,calib_config);
             
        % Get sub pixel calibration board points -------------------------%     
        
        % For first iteration, initialize homographies from world to pixel
        % coordinates using four point boxes
        if it == 1
            for i = 1:num_boards
                % Get four point box in pixel coordinates
                if exist('a','var') && exist('d','var')
                    % If intrinsics are passed in, then undistort four
                    % point box
                    p_fp_ps = alg.p_p_d2p_p(p_fp_p_dss{i}, ...
                                            p_fp_p_dss{i}, ... % Use distorted points for initial guess
                                            f_p_p2p_p_d, ...
                                            f_dp_p_d_dargs{1}, ... % x_p
                                            f_dp_p_d_dargs{2}, ... % y_p                                            
                                            a, ...
                                            d, ...
                                            calib_config);
                else
                    % If intrinsics arent available, assume distortion is small
                    p_fp_ps = p_fp_p_dss{i};
                end
                
                % Compute homography
                H_w2ps{i} = alg.homography_p2p(p_fp_ws, ...
                                               p_fp_ps, ...
                                               calib_config); %#ok<AGROW>
            end
        end
        
        % Get calibration board points
        util.verbose_disp('---',2,calib_config);
        for i = 1:num_boards   
            t = tic;
            util.verbose_fprintf(['Refining "' calib_config.target_type '" points for: ' img_cbs(i).get_path() '. '],2,calib_config);

            % Get undistorted calibration board image array and the 
            % transform from world to pixel coordinates.
            if exist('a','var') && exist('d','var')
                % undistort array
                array_cb = alg.undistort_array(img_cbs(i).get_array_gs(), ...
                                               f_p_p2p_p_d, ...
                                               a, ...
                                               d, ...
                                               calib_config);
            else
                % If intrinsics arent available, assume distortion is small
                array_cb = img_cbs(i).get_array_gs();
            end
            f_p_w2p_p = @(p)(alg.apply_homography_p2p(p,H_w2ps{i}));

            % Get point refinement function
            switch calib_config.target_type
                case 'checker'
                    f_refine_points = @alg.refine_checker_points;
                case 'circle'
                    f_refine_points = @alg.refine_circle_points;
                otherwise
                    error(['Unknown target type: "' calib_config.target_type '"']);
            end

            % Refine points
            [p_cb_pss{i}, cov_cb_pss{i}, idx_valids{i}, debugs{i}] = f_refine_points(array_cb, ...
                                                                                     f_p_w2p_p, ...
                                                                                     calib_config, ...
                                                                                     calib_config.target_mat(:)); %#ok<AGROW>
                                                                                          
            time = toc(t);
            util.verbose_fprintf(['Time ellapsed: %f seconds.' newline],time,2,calib_config);           
        end

        % Get initial intrinsic and extrinsic parameters -----------------%     
        
        if it == 1
            % Update homographies using refined points
            for i = 1:num_boards     
                % Get the homography estimation function
                switch calib_config.target_type
                    case 'checker'
                        % Use "point to point" method
                        f_homography = @alg.homography_p2p;
                    case 'circle'
                        % Use "circle to ellipse" method
                        f_homography = @alg.homography_c2e;                
                    otherwise
                        error(['Unknown target type: "' calib_config.target_type '"']);
                end 

                % Compute homography 
                if calib_config.apply_covariance_optimization
                    % Get sparse covariance matrix
                    cov_cb_p_sparse = cellfun(@sparse,cov_cb_pss{i}(idx_valids{i}),'UniformOutput',false);
                    cov_cb_p_sparse = blkdiag(cov_cb_p_sparse{:});
                    
                    % Use covariance matrix in homography estimation
                    H_w2ps{i} = f_homography(p_cb_ws(idx_valids{i},:), ...
                                             p_cb_pss{i}(idx_valids{i},:), ...
                                             calib_config, ...
                                             cov_cb_p_sparse); % This is full covariance matrix
                else
                    H_w2ps{i} = f_homography(p_cb_ws(idx_valids{i},:), ...
                                             p_cb_pss{i}(idx_valids{i},:), ...
                                             calib_config); 
                end
            end    

            % Get initial guess for camera matrix
            if ~exist('a','var')
                a = alg.A2a(alg.init_intrinsic_params(H_w2ps, ...
                                                      img_cbs(1).get_width(), ...
                                                      img_cbs(1).get_height())); 
            end

            % Get initial guess for extrinsics    
            for i = 1:num_boards
                [Rs{i}, ts{i}] = alg.init_extrinsic_params(H_w2ps{i}, alg.a2A(a)); %#ok<AGROW>
            end
            
            % Get initial distortion parameters
            if ~exist('d','var')
                d = zeros(num_params_d,1);
            end
        end
        
        % Apply distortion to points and covariances ---------------------%

        % Update points
        for i = 1:num_boards
            p_cb_p_dss{i} = alg.p_p2p_p_d(p_cb_pss{i}, ...
                                          f_p_p2p_p_d, ...                                          
                                          a, ...
                                          d); %#ok<AGROW>
        end
        
        % Update covariances
        for i = 1:num_boards
            for j = 1:numel(cov_cb_pss{i})
                % Initialize updated covariance - this ensures number of 
                % elements match cov_cb_pss
                cov_cb_p_dss{i}{j,1} = []; %#ok<AGROW>
                if idx_valids{i}(j)
                    % Use taylor series to approximate covariance of
                    % distorted coordinates; from: 
                    %
                    %   http://www.stat.cmu.edu/~hseltman/files/ratio.pdf
                    %   https://en.wikipedia.org/wiki/Propagation_of_uncertainty#Non-linear_combinations
                    %
                    % Note that only covariances from x_p and y_p are used
                    p_cb_p = p_cb_pss{i}(j,:);
                    cov_cb_p = cov_cb_pss{i}{j};
                    
                    % Get Jacobian
                    dp_p_d_dp_p = full(alg.dp_p_d_dp_p(p_cb_p, ...
                                                       f_dp_p_d_dargs{1}, ... % x_p
                                                       f_dp_p_d_dargs{2}, ... % y_p
                                                       a, ...
                                                       d));
                    
                    % Update covariance
                    cov_cb_p_dss{i}{j} = dp_p_d_dp_p*cov_cb_p*dp_p_d_dp_p'; %#ok<AGROW>
                end
            end  
        end
        
        % Perform nonlinear refinement of parameters ---------------------%     
        
        util.verbose_disp('---',3,calib_config);
        util.verbose_disp('Refining single parameters...',3,calib_config);
        
        % Get transform that converts world points to pixel points and its
        % corresponding derivatives wrt homography
        switch calib_config.target_type
            case 'checker'
                % Use "point to point" method
                f_p_w2p_p = @(p,H)alg.apply_homography_p2p(p,H);
                f_dp_p_dh = @(p,H)alg.dp_dh_p2p(p,H);
            case 'circle'
                % Use "circle to ellipse" method
                f_p_w2p_p = @(p,H)alg.apply_homography_c2e(p,H,calib_config.circle_radius);       
                f_dp_p_dh = @(p,H)alg.dp_dh_c2e(p,H,calib_config.circle_radius);         
            otherwise
                error(['Unknown target type: "' calib_config.target_type '"']);
        end
        
        % Gather params
        num_params = 3+num_params_d+6*num_boards;
        params = zeros(num_params,1);
        % Intrinsics
        params(1:3) = a;
        params(4:3+num_params_d) = d;
        % Extrinsics
        for i = 1:num_boards
            params(3+num_params_d+6*(i-1)+1: ...
                   3+num_params_d+6*(i-1)+3) = alg.rot2euler(Rs{i});
            params(3+num_params_d+6*(i-1)+4: ...
                   3+num_params_d+6*(i-1)+6) = ts{i};        
        end
        
        % Refine params
        if calib_config.apply_covariance_optimization
            [params,cov_params] = alg.refine_single_params(params, ...
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
            [params,cov_params] = alg.refine_single_params(params, ...
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
        a = params(1:3);
        d = params(4:3+num_params_d);
        for i = 1:num_boards
            Rs{i} = alg.euler2rot(params(3+num_params_d+6*(i-1)+1: ...
                                         3+num_params_d+6*(i-1)+3));
            ts{i} = params(3+num_params_d+6*(i-1)+4: ...
                           3+num_params_d+6*(i-1)+6);
        end    
        
        % Print params  
        util.verbose_disp('---',1,calib_config);
        util.verbose_disp('Single intrinsic params (+- 3*sigma):',1,calib_config);
        util.verbose_disp('  -Camera: ',1,calib_config);
        print_param('alpha', 1, params, cov_params, newline, calib_config); 
        print_param('x_o',   2, params, cov_params, newline, calib_config);
        print_param('y_o',   3, params, cov_params, newline, calib_config);         
        util.verbose_disp('  -Distortion: ',1,calib_config);
        for i = 1:num_params_d
            print_param(char(args_p_p2p_p_d(i+5)), i+3, params, cov_params, newline, calib_config);
        end
                                           
        % Update homographies --------------------------------------------%
        
        for i = 1:num_boards
            H_w2ps{i} = alg.a2A(a)*[Rs{i}(:,1) Rs{i}(:,2) ts{i}]; 
        end
    end
    
    % Package outputs ----------------------------------------------------%
    calib.intrin.A = alg.a2A(a);
    calib.intrin.d = d;
    for i = 1:num_boards
        calib.extrin(i).img_path = img_cbs(i).get_path();
        calib.extrin(i).R = Rs{i};
        calib.extrin(i).t = ts{i};
        calib.extrin(i).p_fp_p_ds = p_fp_p_dss{i};
        calib.extrin(i).p_cb_p_ds = p_cb_p_dss{i};
        calib.extrin(i).cov_cb_p_ds = cov_cb_p_dss{i};
        calib.extrin(i).idx_valid = idx_valids{i};
        calib.extrin(i).debug = debugs{i};
    end
    calib.config = calib_config;
    calib.debug.params = params;
    calib.debug.cov_params = cov_params;
end

function print_param(s_param, idx, params, cov_params, suffix, calib_config)
    util.verbose_fprintf([pad(['    ' s_param ': '],13) sprintf('% 9.4f',params(idx)) ' +- ' sprintf('% 6.4f',3*sqrt(cov_params(idx,idx))) suffix],1,calib_config);
end