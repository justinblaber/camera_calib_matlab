function calib = single_calib_four_points(cb_imgs,four_points_ps,calib_config,intrin)
    % Performs camera calibration (mostly from Zhang's paper, some stuff
    % adapted from Bouguet's toolbox, and some stuff I've added myself)
                                                             
    disp('---');
    disp('Performing single calibration...');       
        
    % Get calibration board points and four point boxes in world coordinates
    [board_points_w, four_points_w] = alg.cb_points(calib_config);
    
    % Initialize homographies using four point boxes in pixel coordinates %
    for i = 1:length(cb_imgs)
        homographies{i} = alg.homography_p2p(four_points_w, ...
                                             four_points_ps{i}, ...
                                             calib_config); %#ok<AGROW>
    end
        
    % Perform distortion refinement iterations
    for it = 1:calib_config.distortion_refinement_it_cutoff
        disp(['Performing distortion refinement iteration: ' num2str(it) '...']);
    
        % Get sub-pixel calibration board points -------------------------%
        disp('---');
        for i = 1:length(cb_imgs)    
            t = tic;
            fprintf('Refining "%s" points for: %s. ',calib_config.calibration_target,cb_imgs(i).get_path());

            % Get calibration board image array and the transform from 
            % world to pixel coordinates.
            if it == 1
                % For first iteration, use original image and homography to 
                % approximate the transform from world to undistorted pixel 
                % coordinates; this assumes low distortion.
                array = cb_imgs(i).get_array_gs();
                f_xfm_w2p = @(p)(alg.apply_homography_p2p(homographies{i},p));
            else
                % For iterations after first, correct the calibration 
                % board image for distortion and use the transform from 
                % world to undistorted pixel coordinates.
                error('Distortion refinement not implemented yet!');
            end

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
            [board_points_ps{i}, board_covs_ps{i}, idx_valids{i}, debug{i}] = f_refine_points(array, ...
                                                                                              f_xfm_w2p, ...
                                                                                              calib_config, ...
                                                                                              calib_config.target_mat(:)); %#ok<AGROW>

            % If distortion refinement was used, then apply distortion to 
            % points and covariances to bring them into distorted pixel 
            % coordinates
            if it > 1
                error('Distortion refinement not implemented yet!');
                % Apply distortion to points
                
                % Use taylor approximation and distortion jacobian to
                % estimate new covariances
                
            end
                                                                                          
            time = toc(t);
            fprintf(['Time ellapsed: %f seconds.' newline],time);        
        end

        % Update homographies using refined points -----------------------%
        for i = 1:length(cb_imgs)     
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
                homographies{i} = f_homography(board_points_w(idx_valids{i},:), ...
                                               board_points_ps{i}(idx_valids{i},:), ...
                                               calib_config, ...
                                               blkdiag(board_covs_ps{i}{idx_valids{i}})); % This is full covariance matrix
            else
                homographies{i} = f_homography(board_points_w(idx_valids{i},:), ...
                                               board_points_ps{i}(idx_valids{i},:), ...
                                               calib_config); 
            end
        end    

        % Get initial guess for camera matrix
        if exist('intrin','var')
            A_init = intrin.A;
        else
            % Use homographies to obtain initial guess for camera matrix
            A_init = alg.init_intrinsic_params(homographies, ...
                                               cb_imgs(1).get_width(), ...
                                               cb_imgs(1).get_height()); 
        end
                
        % Get initial guess for extrinsic camera parameters (R and t) per homography
        rotations = {};
        translations = {};
        for i = 1:length(cb_imgs)
            [rotations{i}, translations{i}] = alg.init_extrinsic_params(homographies{i}, ...
                                                                        A_init, ...
                                                                        board_points_ps{i}, ...
                                                                        calib_config); %#ok<AGROW>
        end

        % Perform nonlinear refinement of all parameters -----------------%

        % Get initial guess for distortions
        if exist('intrin','var')
            distortion_init = intrin.distortion;
        else
            % Assume zero lens distortion
            distortion_init = zeros(4,1);
        end

        % Perform optimization    
        if exist('intrin','var')
            optimization_type = 'extrinsic';
        else
            optimization_type = 'full';
        end

        disp('---');
        disp('Refining single parameters...');
        [A,distortion,rotations,translations] = alg.refine_single_params(A_init, ... 
                                                                         distortion_init, ...
                                                                         rotations, ...
                                                                         translations, ...
                                                                         board_points_ps, ...
                                                                         optimization_type, ...
                                                                         calib_config);  

        disp('---');
        disp('Initial camera matrix: ');
        disp(A_init);
        disp('Refined camera matrix: ');
        disp(A);   
        disp('Initial distortions: ');
        disp(distortion_init');
        disp('Refined distortions: ');
        disp(distortion');

        % Package outputs ----------------------------------------------------%
        calib.config = calib_config;
        calib.intrin.A = A;
        calib.intrin.distortion = distortion;
        for i = 1:length(cb_imgs)
            calib.extrin(i).cb_img = cb_imgs(i);
            calib.extrin(i).rotation = rotations{i};
            calib.extrin(i).translation = translations{i};
            calib.extrin(i).four_points_p = four_points_ps{i};
            calib.extrin(i).board_points_p = board_points_ps{i};
            calib.extrin(i).debug.homography_refine = homographies_refine{i};
        end
    end
end
