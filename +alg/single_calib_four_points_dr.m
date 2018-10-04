function calib = single_calib_four_points_dr(img_cbs,p_fp_p_dss,calib_config,intrin)
    % Performs camera calibration using "distortion refinement" method.
    
    disp('---');
    disp('Performing single calibration with distortion refinement...');
        
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
    
    % Get function handles for distortion function and its derivatives    
    f_p_p_d = matlabFunction(calib_config.sym_p_p_d);
    f_dp_p_d_dx_p_bar = matlabFunction(alg.diff_vars(calib_config.sym_p_p_d,'x_p_bar'));  
    f_dp_p_d_dy_p_bar = matlabFunction(alg.diff_vars(calib_config.sym_p_p_d,'y_p_bar'));  
    
    % Set number of distortion arguments
    num_args_distortion = nargin(f_p_p_d) - 5;
    
    % Perform distortion refinement iterations, which iteratively applies
    % distortion correction to images to help refine target point locations
    for it = 1:distortion_refinement_it_cutoff
        disp(['Performing distortion refinement iteration: ' num2str(it) '...']);
             
        % Get sub pixel calibration board points -------------------------%     
        
        % For first iteration, initialize homographies from world to pixel
        % coordinates using four point boxes
        if it == 1
            for i = 1:numel(img_cbs) 
                % Get four point box in pixel coordinates
                if exist('A','var') && exist('d','var')
                    % Apply inverse distortion to "four points" if
                    % intrinsics are already passed in
                    p_fp_ps = alg.apply_inv_p_p_d_f(f_p_p_d, ...
                                                    f_dp_p_d_dx_p_bar, ...
                                                    f_dp_p_d_dy_p_bar, ...
                                                    p_fp_p_dss{i}, ... % Use distorted points for initial guess
                                                    p_fp_p_dss{i}, ...
                                                    A, ...
                                                    d, ...
                                                    calib_config);
                else
                    % Assume small distortion if intrinsics aren't
                    % initially provided
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
        for i = 1:numel(img_cbs)    
            t = tic;
            fprintf(['Refining "' calib_config.calibration_target '" points for: ' img_cbs(i).get_path() '. ']);

            % Get undistorted calibration board image array and the 
            % transform from world to pixel coordinates.
            if exist('A','var') && exist('d','var')
                % "undo" distortion on image
                array_cb = alg.apply_inv_p_p_d_array(img_cbs(i).get_array_gs(), ...
                                                     f_p_p_d, ...
                                                     A, ...
                                                     d, ...
                                                     calib_config);
            else
                % If intrinsics arent available, assume distortion is small
                array_cb = img_cbs(i).get_array_gs();
            end
            f_xfm_w2p = @(p)(alg.apply_homography_p2p(H_w2ps{i},p));

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
                                                                                     f_xfm_w2p, ...
                                                                                     calib_config, ...
                                                                                     calib_config.target_mat(:)); %#ok<AGROW>
                                                                                          
            time = toc(t);
            fprintf(['Time ellapsed: %f seconds.' newline],time);        
        end

        % Get initial intrinsic and extrinsic parameters -----------------%     
        
        if it == 1
            % Update homographies using refined points
            for i = 1:numel(img_cbs)     
                % Get the homography estimation function
                switch calib_config.target_type
                    case 'checker'
                        % Use "point to point" homography estimation
                        f_homography = @alg.homography_p2p;
                    case 'circle'
                        % Use "circle to ellipse" homography estimation
                        f_homography = @alg.homography_c2e;                
                    otherwise
                        error(['Unknown target type: "' calib_config.target_type '"']);
                end 

                % Compute homography 
                if calib_config.apply_covariance_optimization
                    % Use covariance matrix in homography estimation
                    H_w2ps{i} = f_homography(p_cb_ws(idx_valids{i},:), ...
                                             p_cb_pss{i}(idx_valids{i},:), ...
                                             calib_config, ...
                                             blkdiag(cov_cb_pss{i}{idx_valids{i}})); % This is full covariance matrix
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
            Rs = {};
            ts = {};
            for i = 1:numel(img_cbs)
                [Rs{i}, ts{i}] = alg.init_extrinsic_params(H_w2ps{i}, A); %#ok<AGROW>
            end
            
            % Get initial distortion parameters
            if ~exist('d','var')
                d = zeros(num_args_distortion,1);
            end
        end
        
        % Apply distortion to points and covariances ---------------------%
                
        % Update points
        for i = 1:numel(p_cb_pss)
            p_cb_p_dss{i} = alg.apply_p_p_d_f(f_p_p_d,p_cb_pss{i},A,d); %#ok<AGROW>
        end
        
        % Update covariances
        for i = 1:numel(cov_cb_pss)
            for j = 1:numel(cov_cb_pss{i})
                % Initialize - this insures number of elements match
                % cov_cb_pss
                cov_cb_p_dss{i}{j,1} = []; %#ok<AGROW>
                if idx_valids{i}(j)
                    % Use taylor series to approximate covariance of
                    % distorted coordinates
                    % From: http://www.stat.cmu.edu/~hseltman/files/ratio.pdf
                    % Note that only covariances from x_p and y_p are used
                    p_cb_p = p_cb_pss{i}(j,:);
                    cov_cb_p = cov_cb_pss{i}{j};
                    
                    % Get derivatives
                    dp_p_d_dp_p_bar = vertcat(alg.apply_p_p_d_f(f_dp_p_d_dx_p_bar,p_cb_p,A,d), ...
                                              alg.apply_p_p_d_f(f_dp_p_d_dy_p_bar,p_cb_p,A,d));
                    
                    % Update covariances
                    var_x_p_d = dp_p_d_dp_p_bar(1,1)^2*cov_cb_p(1,1) + ...
                                2*dp_p_d_dp_p_bar(1,1)*dp_p_d_dp_p_bar(2,1)*cov_cb_p(1,2) + ...
                                dp_p_d_dp_p_bar(2,1)^2*cov_cb_p(2,2);
                    var_y_p_d = dp_p_d_dp_p_bar(1,2)^2*cov_cb_p(1,1) + ...
                                2*dp_p_d_dp_p_bar(1,2)*dp_p_d_dp_p_bar(2,2)*cov_cb_p(1,2) + ...
                                dp_p_d_dp_p_bar(2,2)^2*cov_cb_p(2,2);
                    cov_x_p_d_y_p_d = dp_p_d_dp_p_bar(1,1)*dp_p_d_dp_p_bar(1,2)*cov_cb_p(1,1) + ...
                                      dp_p_d_dp_p_bar(1,1)*dp_p_d_dp_p_bar(2,2)*cov_cb_p(1,2) + ... 
                                      dp_p_d_dp_p_bar(2,1)*dp_p_d_dp_p_bar(1,2)*cov_cb_p(1,2) + ...
                                      dp_p_d_dp_p_bar(2,1)*dp_p_d_dp_p_bar(2,2)*cov_cb_p(2,2);
                                  
                    % Store covariance
                    cov_cb_p_dss{i}{j} = [var_x_p_d       cov_x_p_d_y_p_d;
                                          cov_x_p_d_y_p_d var_y_p_d]; %#ok<AGROW>
                end
            end  
        end
        
        % Perform nonlinear refinement of all parameters -----------------%     
        
        disp('---');
        disp('Refining single parameters...');
                
        if calib_config.apply_covariance_optimization
            [A,d,Rs,ts] = alg.refine_single_params(A, ... 
                                                   d, ...
                                                   Rs, ...
                                                   ts, ...
                                                   p_cb_p_dss, ...
                                                   optimization_type, ...
                                                   calib_config);
        else
            [A,d,Rs,ts] = alg.refine_single_params(A, ... 
                                                   d, ...
                                                   Rs, ...
                                                   ts, ...
                                                   p_cb_p_dss, ...
                                                   optimization_type, ...
                                                   calib_config, ...
                                                   cov_cb_p_dss);
        end
                                           
        disp('---');
        disp('Refined camera matrix: ');
        disp(A);   
        disp('Refined distortions: ');
        disp(d');  
                                           
        % Update homographies --------------------------------------------%
        
        for i = 1:numel(H_w2ps) 
            H_w2ps{i} = A*[Rs{i}(:,1) Rs{i}(:,2) ts{i}]; 
        end
    end
    
    % Package outputs ----------------------------------------------------%
    calib.config = calib_config;
    calib.intrin.A = A;
    calib.intrin.d = d;
    for i = 1:numel(img_cbs)
        calib.extrin(i).cb_img = img_cbs(i);
        calib.extrin(i).rotation = Rs{i};
        calib.extrin(i).translation = ts{i};
        calib.extrin(i).four_points_p = p_fp_p_dss{i};
        calib.extrin(i).board_points_p = p_cb_pss{i};
        calib.extrin(i).debug.homography_refine = homographies_refine{i};
    end
end
