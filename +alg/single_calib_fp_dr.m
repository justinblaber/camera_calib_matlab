function calib = single_calib_fp_dr(img_cbs,p_fp_p_dss,calib_config,intrin)
    % Performs camera calibration using "four point distortion refinement"
    % method.
    
    disp('---');
    disp('Performing single calibration with four point distortion refinement method...');
        
    % If intrinsics are passed in, don't optimize for them
    if exist('intrin','var')
        A = intrin.A;
        d = intrin.d;
        optimization_type = 'extrinsic';
        distortion_refinement_it_cutoff = 1;
    else
        optimization_type = 'full';
        distortion_refinement_it_cutoff = calib_config.distortion_refinement_it_cutoff;
    end
    
    % Get the calibration board points and the four point box on the 
    % calibration board in world coordinates.
    [p_cb_ws, p_fp_ws] = alg.p_cb_w(calib_config);
                
    % Get number of boards
    num_boards = numel(img_cbs);
    
    % Get function handle for distortion function   
    f_p_p2p_p_d = matlabFunction(calib_config.sym_p_p2p_p_d);
    
    % Get function handles for distortion function partial derivatives
    args_p_p2p_p_d = argnames(calib_config.sym_p_p2p_p_d);
    for i = 1:length(args_p_p2p_p_d)
        % Differentiate
        f_dp_p_d_dargs{i} = diff(calib_config.sym_p_p2p_p_d, ...
                                 args_p_p2p_p_d(i)); %#ok<AGROW>
        % Convert to function handle 
        f_dp_p_d_dargs{i} = matlabFunction(f_dp_p_d_dargs{i}); %#ok<AGROW>
    end    
        
    % Get number of distortion params    
    num_params_d = nargin(f_p_p2p_p_d) - 5;
    
    % Perform distortion refinement iterations, which iteratively applies
    % distortion correction to images to help refine target point locations
    for it = 1:distortion_refinement_it_cutoff
        disp(['Performing distortion refinement iteration: ' num2str(it) '...']);
             
        % Get sub pixel calibration board points -------------------------%     
        
        % For first iteration, initialize homographies from world to pixel
        % coordinates using four point boxes
        if it == 1
            for i = 1:num_boards
                % Get four point box in pixel coordinates
                if exist('A','var') && exist('d','var')
                    % If intrinsics are passed in, then undistort four
                    % point box
                    p_fp_ps = alg.p_p_d2p_p(p_fp_p_dss{i}, ...
                                            p_fp_p_dss{i}, ... % Use distorted points for initial guess
                                            f_p_p2p_p_d, ...
                                            f_dp_p_d_dargs{1}, ... % x_p
                                            f_dp_p_d_dargs{2}, ... % y_p                                            
                                            A, ...
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
        disp('---');
        for i = 1:num_boards   
            t = tic;
            fprintf(['Refining "' calib_config.calibration_target '" points for: ' img_cbs(i).get_path() '. ']);

            % Get undistorted calibration board image array and the 
            % transform from world to pixel coordinates.
            if exist('A','var') && exist('d','var')
                % undistort array
                array_cb = alg.undistort_array(img_cbs(i).get_array_gs(), ...
                                               f_p_p2p_p_d, ...
                                               A, ...
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
            fprintf(['Time ellapsed: %f seconds.' newline],time);        
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
            if ~exist('A','var')
                A = alg.init_intrinsic_params(H_w2ps, ...
                                              img_cbs(1).get_width(), ...
                                              img_cbs(1).get_height()); 
            end

            % Get initial guess for extrinsics    
            for i = 1:num_boards
                [Rs{i}, ts{i}] = alg.init_extrinsic_params(H_w2ps{i}, A); %#ok<AGROW>
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
                                          A, ...
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
                    %
                    % Note that only covariances from x_p and y_p are used
                    p_cb_p = p_cb_pss{i}(j,:);
                    cov_cb_p = cov_cb_pss{i}{j};
                    
                    % Get derivatives
                    dp_p_d_dp_p = alg.dp_p_d_dp_p(p_cb_p, ...
                                                  f_dp_p_d_dargs{1}, ... % x_p
                                                  f_dp_p_d_dargs{2}, ... % y_p
                                                  A, ...
                                                  d);
                    dp_p_d_dp_p = full(dp_p_d_dp_p); % dp_p_d_dp_p is sparse
                    
                    % Update covariances
                    var_x_p_d = dp_p_d_dp_p(1,1)^2*cov_cb_p(1,1) + ...
                                2*dp_p_d_dp_p(1,1)*dp_p_d_dp_p(1,2)*cov_cb_p(1,2) + ...
                                dp_p_d_dp_p(1,2)^2*cov_cb_p(2,2);
                            
                    var_y_p_d = dp_p_d_dp_p(2,1)^2*cov_cb_p(1,1) + ...
                                2*dp_p_d_dp_p(2,1)*dp_p_d_dp_p(2,2)*cov_cb_p(1,2) + ...
                                dp_p_d_dp_p(2,2)^2*cov_cb_p(2,2);
                            
                    cov_x_p_d_y_p_d = dp_p_d_dp_p(1,1)*dp_p_d_dp_p(2,1)*cov_cb_p(1,1) + ...
                                      dp_p_d_dp_p(1,1)*dp_p_d_dp_p(2,2)*cov_cb_p(1,2) + ... 
                                      dp_p_d_dp_p(1,2)*dp_p_d_dp_p(2,1)*cov_cb_p(1,2) + ...
                                      dp_p_d_dp_p(1,2)*dp_p_d_dp_p(2,2)*cov_cb_p(2,2);
                                  
                    % Store covariance
                    cov_cb_p_dss{i}{j} = [var_x_p_d       cov_x_p_d_y_p_d;
                                          cov_x_p_d_y_p_d var_y_p_d]; %#ok<AGROW>
                end
            end  
        end
        
        % Perform nonlinear refinement of parameters ---------------------%     
        
        disp('---');
        disp('Refining single parameters...');
        
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
        
        if calib_config.apply_covariance_optimization
            [A,d,Rs,ts] = alg.refine_single_params(A, d, Rs, ts, ...
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
            [A,d,Rs,ts] = alg.refine_single_params(A, d, Rs, ts, ...
                                                   p_cb_p_dss, ...
                                                   idx_valids, ...  
                                                   f_p_w2p_p, ...
                                                   f_dp_p_dh, ...
                                                   f_p_p2p_p_d, ...
                                                   f_dp_p_d_dargs, ...
                                                   optimization_type, ...  
                                                   calib_config);
        end
                                           
        disp('---');
        disp('Refined camera matrix: ');
        disp(A);   
        disp('Refined distortions: ');
        disp(d');  
                                           
        % Update homographies --------------------------------------------%
        
        for i = 1:num_boards
            H_w2ps{i} = A*[Rs{i}(:,1) Rs{i}(:,2) ts{i}]; 
        end
    end
    
    % Package outputs ----------------------------------------------------%
    calib.config = calib_config;
    calib.intrin.A = A;
    calib.intrin.d = d;
    for i = 1:num_boards
        calib.extrin(i).img_cb = img_cbs(i);
        calib.extrin(i).R = Rs{i};
        calib.extrin(i).t = ts{i};
        calib.extrin(i).p_fp_p_ds = p_fp_p_dss{i};
        calib.extrin(i).p_cb_ps = p_cb_pss{i};
        calib.extrin(i).cov_cb_ps = cov_cb_pss{i};
        calib.extrin(i).idx_valid = idx_valids{i};
        calib.extrin(i).debug = debugs{i};
    end
end