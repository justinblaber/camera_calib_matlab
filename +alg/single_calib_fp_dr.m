function calib = single_calib_fp_dr(obj_A, obj_R, obj_cb_w2p, obj_distortion, obj_cb_geom, img_cbs, p_fp_p_dss, calib_config, intrin)
    % Performs camera calibration using "four point distortion refinement"
    % method.
    %
    % Inputs:
    %   obj_A - class.calib.A_intf; parameterization for camera matrix
    %   obj_R - class.calib.R_intf; parameterization for rotation matrix
    %   obj_cb_w2p - class.calib.cb_w2p_intf; describes the mapping between
    %       calibration board world points and calibration board pixel
    %       points.
    %   obj_distortion - class.distortion.intf; describes the mapping
    %       between pixel coordinates and distorted pixel coordinates.
    %   obj_cb_geom - class.cb_geom.fp_intf & class.cb_geom.target_intf;
    %       calibration board four-point and target geometry interface.
    %   img_cbs - class.img.intf; Nx1 calibration board image interfaces.
    %   p_fp_p_dss - cell; Nx1 cell of four point boxes around the
    %       calibration board images in distorted pixel coordinates
    %   calib_config - struct; struct returned by intf.load_calib_config()
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
    %           .img_cb - class.img; calibration board image
    %           .R - array; 3x3 rotation matrix
    %           .t - array; 3x1 translation vector
    %           .p_fp_p_ds - array; four point box around the calibration
    %               board image in distorted pixel coordinates
    %           .p_cb_p_ds - array; calibration board distorted pixel
    %               points
    %           .cov_cb_p_ds - cell; covariances of calibration board
    %               distorted pixel points
    %           .p_cb_p_d_ms - array; calibration board model distorted
    %               pixel points
    %           .idx_valid - array; valid calibration board points
    %           .debug - cell;
    %       .debug - struct;

    util.verbose_disp('------', 1, calib_config);
    util.verbose_disp('Performing single calibration with four point distortion refinement method...', 1, calib_config);

    % Perform single calibration -----------------------------------------%
    
    % Get calibration object
    obj_single_calib = class.calib.single(obj_A, ...
                                          obj_R, ...
                                          obj_cb_w2p, ...
                                          obj_distortion, ...
                                          calib_config);

    % Get the calibration board points and boundaries in world coordinates
    p_fp_ws = obj_cb_geom.get_p_fp_ws();
    p_cb_ws = obj_cb_geom.get_p_cb_ws();
    boundary_ws = obj_cb_geom.get_p_cb_w_boundaries();

    % Get number of boards
    num_boards = numel(img_cbs);

    % Get optimization type
    if exist('intrin', 'var')
        optimization_type = 'extrinsic'; % Only optimize extrinsics

        % Also parse intrinsics
        A = intrin.A;
        d = intrin.d;
    else
        optimization_type = 'full';      % Optimize intrinsics and extrinsics
    end

    % Get distortion refinement iterations
    if exist('intrin', 'var') || obj_single_calib.get_num_params_d() == 0
        % Do not perform distortion refinement if intrinsics are already
        % passed in or if distortion function has no distortion parameters.
        distortion_refinement_it_cutoff = 1;
    else
        distortion_refinement_it_cutoff = calib_config.distortion_refinement_it_cutoff;
    end

    % Iterate
    for it = 1:distortion_refinement_it_cutoff
        util.verbose_disp('---', 1, calib_config);
        util.verbose_disp(['Performing distortion refinement iteration: ' num2str(it) '...'], 1, calib_config);

        % Get calibration board pixel points -----------------------------%

        % For first iteration, initialize homographies from world to pixel
        % coordinates using four point boxes
        if it == 1
            for i = 1:num_boards
                % Get four point box in pixel coordinates
                if exist('A', 'var') && exist('d', 'var')
                    % If intrinsics are passed in, undistort four point box
                    p_fp_ps = obj_single_calib.p_p_d2p_p(p_fp_p_dss{i}, ...
                                                         p_fp_p_dss{i}, ...     % Use distorted points for initial guess
                                                         A, ...
                                                         d);
                else
                    % If intrinsics arent available, assume distortion is small
                    p_fp_ps = p_fp_p_dss{i};
                end

                % Compute homography - use direct p2p method
                H_w2ps{i} = alg.homography_p2p(p_fp_ws, ...
                                               p_fp_ps, ...
                                               calib_config); %#ok<AGROW>
            end
        end

        % Get calibration board points
        util.verbose_disp('---', 2, calib_config);
        for i = 1:num_boards
            t = tic;
            util.verbose_fprintf(['Refining "' calib_config.target '" points with method: "' calib_config.target_optimization '" for: ' img_cbs(i).get_path() '. '], 2, calib_config);

            % Get undistorted calibration board image array
            if exist('A', 'var') && exist('d', 'var')
                % undistort array
                array_cb = alg.undistort_array(img_cbs(i).get_array_gs(), ...
                                               obj_single_calib.get_obj_distortion(), ...
                                               A, ...
                                               d, ...
                                               calib_config);
            else
                % If intrinsics arent available, assume distortion is small
                array_cb = img_cbs(i).get_array_gs();
            end

            % Refine points
            [p_cb_pss{i}, cov_cb_pss{i}, idx_valids{i}, debug{i}] = alg.refine_target_points_cb_w2p(p_cb_ws, ...
                                                                                                    boundary_ws, ...
                                                                                                    array_cb, ...
                                                                                                    @(p)obj_single_calib.p_cb_w2p_cb_p(p, H_w2ps{i}), ...
                                                                                                    @(p)alg.apply_homography_p2p(p, H_w2ps{i}), ...
                                                                                                    calib_config); %#ok<AGROW>

            time = toc(t);
            util.verbose_fprintf(['Time ellapsed: %f seconds.' newline], time, 2, calib_config);
        end

        % Get initial intrinsic and extrinsic parameters -----------------%

        if it == 1
            % First, update homographies using calibration board pixel points
            for i = 1:num_boards
                if calib_config.apply_covariance_optimization
                    % Get sparse covariance matrix
                    cov_cb_p_sparse = cellfun(@sparse, cov_cb_pss{i}(idx_valids{i}), 'UniformOutput', false);
                    cov_cb_p_sparse = blkdiag(cov_cb_p_sparse{:});

                    H_w2ps{i} = obj_single_calib.homography_cb_w2p(p_cb_ws(idx_valids{i}, :), ...
                                                                   p_cb_pss{i}(idx_valids{i}, :), ...
                                                                   cov_cb_p_sparse);
                else
                    H_w2ps{i} = obj_single_calib.homography_cb_w2p(p_cb_ws(idx_valids{i}, :), ...
                                                                   p_cb_pss{i}(idx_valids{i}, :));
                end
            end

            % Get initial guess for camera matrix
            if ~exist('A', 'var')
                A = alg.init_intrinsic_params(H_w2ps, ...
                                              img_cbs(1).get_width(), ...
                                              img_cbs(1).get_height());
            end

            % Get initial guess for extrinsics
            for i = 1:num_boards
                [Rs{i}, ts{i}] = alg.init_extrinsic_params(H_w2ps{i}, A); %#ok<AGROW>
            end

            % Get initial distortion parameters
            if ~exist('d', 'var')
                d = zeros(obj_single_calib.get_num_params_d(), 1);
            end
        end

        % ----------------------------------------------------------------%
        % Apply distortion to calibration board pixel points and
        % covariances

        for i = 1:num_boards
            % Update points
            p_cb_p_dss{i} = obj_single_calib.p_p2p_p_d(p_cb_pss{i}, A, d); %#ok<AGROW>

            % Update covariances
            for j = 1:numel(cov_cb_pss{i})
                % Initialize updated covariance - this ensures number of
                % elements match cov_cb_pss
                cov_cb_p_dss{i}{j, 1} = []; %#ok<AGROW>
                if idx_valids{i}(j)
                    % Use taylor series to approximate covariance of
                    % distorted coordinates; from:
                    %
                    %   http://www.stat.cmu.edu/~hseltman/files/ratio.pdf
                    %   https://en.wikipedia.org/wiki/Propagation_of_uncertainty#Non-linear_combinations
                    %
                    % Note that only covariances from x_p and y_p are used
                    p_cb_p = p_cb_pss{i}(j, :);
                    cov_cb_p = cov_cb_pss{i}{j};

                    % Get Jacobian
                    dp_p_d_dp_p = obj_single_calib.dp_p_d_dp_p(p_cb_p, A, d);
                    dp_p_d_dp_p = full(dp_p_d_dp_p);

                    % Update covariance
                    cov_cb_p_dss{i}{j} = dp_p_d_dp_p*cov_cb_p*dp_p_d_dp_p'; %#ok<AGROW>
                end
            end
        end

        % Perform refinement of parameters -------------------------------%

        util.verbose_disp('---', 3, calib_config);
        util.verbose_disp(['Refining single parameters with ' optimization_type ' optimization...'], 3, calib_config);

        if calib_config.apply_covariance_optimization
            util.verbose_disp('Applying covariance optimization.', 3, calib_config);

            [A, d, Rs, ts] = obj_single_calib.refine(A, ...
                                                     d, ...
                                                     Rs, ...
                                                     ts, ...
                                                     p_cb_ws, ...
                                                     p_cb_p_dss, ...
                                                     idx_valids, ...
                                                     optimization_type, ...
                                                     cov_cb_p_dss);
        else
            [A, d, Rs, ts] = obj_single_calib.refine(A, ...
                                                     d, ...
                                                     Rs, ...
                                                     ts, ...
                                                     p_cb_ws, ...
                                                     p_cb_p_dss, ...
                                                     idx_valids, ...
                                                     optimization_type);
        end

        % Update homographies --------------------------------------------%

        for i = 1:num_boards
            H_w2ps{i} = obj_single_calib.ARt2H(A, Rs{i}, ts{i});
        end
    end

    % Package outputs ----------------------------------------------------%
    % Config
    calib.config = calib_config;
    % Intrinsics
    calib.intrin.A = A;
    calib.intrin.d = d;
    % Extrinsics
    for i = 1:num_boards
        calib.extrin(i).img_cb = img_cbs(i);
        calib.extrin(i).R = Rs{i};
        calib.extrin(i).t = ts{i};
        calib.extrin(i).p_fp_p_ds = p_fp_p_dss{i};
        calib.extrin(i).p_cb_p_ds = p_cb_p_dss{i};
        calib.extrin(i).cov_cb_p_ds = cov_cb_p_dss{i};
        calib.extrin(i).p_cb_p_d_ms = obj_single_calib.p_cb_w2p_cb_p_d(p_cb_ws, Rs{i}, ts{i}, A, d);
        calib.extrin(i).idx_valid = idx_valids{i};
        calib.extrin(i).debug = debug{i};
    end
end
