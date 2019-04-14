function calib = single_calib_H_fr(obj_calib, obj_cb_geom, img_cbs, H_w2ps, calib_config, intrin)
    % Performs single camera calibration using "homography frontal
    % refinement" method.
    %
    % Inputs:
    %   obj_calib - class.calib.base; calibration object
    %   obj_cb_geom - class.cb_geom.size_intf & class.cb_geom.target_intf;
    %       calibration board size and target geometry interface.
    %   img_cbs - class.img.intf; Nx1 calibration board image interfaces.
    %   H_w2ps - cell; Nx1 cell of initial guesses of homographies which
    %       map world coordinates to pixel coordinates
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %   intrin - struct; optional. If passed in, intrinsics will not be
    %       optimized.
    %       .A - array; camera matrix
    %       .d - array; array of distortion coefficients
    %
    % Outputs:
    %   calib - struct;
    %       .config - struct; copy of input calib_config
    %       .cam - struct;
    %           .intrin - struct;
    %               .A - array; 3x3 camera matrix
    %               .d - array; Mx1 array of distortion coefficients
    %           .extrin - struct; Nx1 struct containing extrinsics
    %               .img_cb - class.img.intf; calibration board image
    %               .R - array; 3x3 rotation matrix
    %               .t - array; 3x1 translation vector
    %               .p_cb_p_ds - array; calibration board distorted pixel
    %                   points
    %               .cov_cb_p_ds - cell; covariances of calibration board
    %                   distorted pixel points
    %               .p_cb_p_d_ms - array; calibration board model distorted
    %                   pixel points
    %               .idx_valid - array; valid calibration board points
    %           .R_1 - array; relative rotation between camera "1" and this
    %               camera
    %           .t_1 - array; relative translation between camera "1" and
    %               this camera

    util.verbose_disp('------', 1, calib_config);
    util.verbose_disp('Performing single calibration with frontal refinement method...', 1, calib_config);

    % Perform single calibration -----------------------------------------%

    % Get the calibration board points and boundaries in world coordinates
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

    % Get frontal refinement iterations
    frontal_refinement_it_cutoff = calib_config.frontal_refinement_it_cutoff;

    % Get frontal array world coordinates --------------------------------%

    % Get samples per unit
    if ~isnan(calib_config.frontal_refinement_samples_per_unit)
        samples_per_unit = calib_config.frontal_refinement_samples_per_unit;
    else
        samples_per_unit = max((img_cbs(1).get_size()-1)./([calib_config.obj_cb_geom.get_cb_height() calib_config.obj_cb_geom.get_cb_width()]));
    end

    % Get number of height and width samples
    num_samples_height = ceil(calib_config.obj_cb_geom.get_cb_height()*samples_per_unit+1);
    num_samples_width = ceil(calib_config.obj_cb_geom.get_cb_width()*samples_per_unit+1);

    % Get height and width of frontal array in world coordinates
    array_frontal_height_w = (num_samples_height-1)/samples_per_unit;
    array_frontal_width_w = (num_samples_width-1)/samples_per_unit;

    % Get samples
    [y_frontal_ws, x_frontal_ws] = ndgrid(linspace(0, array_frontal_height_w, num_samples_height), ...
                                          linspace(0, array_frontal_width_w,  num_samples_width));
    p_frontal_ws = [x_frontal_ws(:) y_frontal_ws(:)];

    % Iterate
    for it = 1:frontal_refinement_it_cutoff
        util.verbose_disp('---', 1, calib_config);
        util.verbose_disp(['Performing frontal refinement iteration: ' num2str(it) '...'], 1, calib_config);
        util.verbose_disp('---', 2, calib_config);

        % Get calibration board pixel points -----------------------------%

        p_cb_pss = cell(num_boards, 1);
        cov_cb_pss = cell(num_boards, 1);
        idx_valids = cell(num_boards, 1);
        debug = cell(num_boards, 1);
        for i = 1:num_boards
            t = tic;
            util.verbose_fprintf(['Refining "' calib_config.target '" points with method: "' calib_config.target_optimization '" for: ' img_cbs(i).get_name() '. '], 2, calib_config);

            % Get frontal calibration board image array
            if exist('A', 'var') && exist('d', 'var')
                p_frontal_p_ds = obj_calib.p_p2p_p_d(alg.apply_homography_p2p(p_frontal_ws, H_w2ps{i}), ...
                                                     A, ...
                                                     d);
            else
                % If intrinsics arent available, assume distortion is small
                p_frontal_p_ds = alg.apply_homography_p2p(p_frontal_ws, H_w2ps{i});
            end

            % Resample array
            array_cb_frontal = alg.interp_array(img_cbs(i).get_array_gs(), ...
                                                p_frontal_p_ds, ...
                                                calib_config.frontal_refinement_interp);
            array_cb_frontal = reshape(array_cb_frontal, num_samples_height, num_samples_width);

            % Refine points
            [p_cb_frontals, cov_cb_frontals, idx_valids{i}, debug{i}] = alg.refine_target_points_cb_w2p(p_cb_ws, ...
                                                                                                        boundary_ws, ...
                                                                                                        array_cb_frontal, ...
                                                                                                        @(p)(p.*samples_per_unit+1), ...
                                                                                                        @(p)(p.*samples_per_unit+1), ...
                                                                                                        calib_config);

            % Convert to pixel coordinates -------------------------------%

            % Update points
            p_cb_pss{i} = obj_calib.p_cb_w2p_cb_p((p_cb_frontals-1)./samples_per_unit, ... % Convert to world coordinates
                                                  H_w2ps{i});

            % Update covariances
            cov_cb_pss{i} = cell(size(p_cb_pss{i}, 1), 1);
            for j = 1:numel(cov_cb_frontals)
                if idx_valids{i}(j)
                    % Use taylor series to approximate covariance of
                    % distorted coordinates; from:
                    %
                    %   http://www.stat.cmu.edu/~hseltman/files/ratio.pdf
                    %   https://en.wikipedia.org/wiki/Propagation_of_uncertainty#Non-linear_combinations
                    %
                    % Note that only covariances from x_cb_w and y_cb_w are used
                    p_cb_w = (p_cb_frontals(j, :)-1)./samples_per_unit;    % Note this is the MEASURED calibration board world point
                    cov_cb_w = cov_cb_frontals{j}./(samples_per_unit^2);   % Must square scaling factor for covariance

                    % Get Jacobian
                    dp_cb_p_dp_cb_w = obj_calib.dp_cb_p_dp_cb_w(p_cb_w, H_w2ps{i});
                    dp_cb_p_dp_cb_w = full(dp_cb_p_dp_cb_w);

                    % Update covariance
                    cov_cb_pss{i}{j} = dp_cb_p_dp_cb_w*cov_cb_w*dp_cb_p_dp_cb_w';
                end
            end

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

                    H_w2ps{i} = obj_calib.homography_cb_w2p(p_cb_ws(idx_valids{i}, :), ...
                                                            p_cb_pss{i}(idx_valids{i}, :), ...
                                                            cov_cb_p_sparse);
                else
                    H_w2ps{i} = obj_calib.homography_cb_w2p(p_cb_ws(idx_valids{i}, :), ...
                                                            p_cb_pss{i}(idx_valids{i}, :));
                end
            end

            % Get initial guess for camera matrix
            if ~exist('A', 'var')
                A = alg.init_intrinsic_params(H_w2ps, img_cbs(1).get_size());
            end

            % Get initial guess for extrinsics
            Rs = cell(num_boards, 1);
            ts = cell(num_boards, 1);
            for i = 1:num_boards
                [Rs{i}, ts{i}] = alg.init_extrinsic_params(H_w2ps{i}, A);
            end

            % Get initial distortion parameters
            if ~exist('d', 'var')
                d = zeros(obj_calib.get_num_params_d(), 1);
            end
        end

        % ----------------------------------------------------------------%
        % Apply distortion to calibration board pixel points and
        % covariances

        p_cb_p_dss = cell(num_boards, 1);
        cov_cb_p_dss = cell(num_boards, 1);
        for i = 1:num_boards
            % Update points
            p_cb_p_dss{i} = obj_calib.p_p2p_p_d(p_cb_pss{i}, A, d);

            % Update covariances
            cov_cb_p_dss{i} = cell(numel(cov_cb_pss{i}), 1);
            for j = 1:numel(cov_cb_pss{i})
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
                    dp_p_d_dp_p = obj_calib.dp_p_d_dp_p(p_cb_p, A, d);
                    dp_p_d_dp_p = full(dp_p_d_dp_p);

                    % Update covariance
                    cov_cb_p_dss{i}{j} = dp_p_d_dp_p*cov_cb_p*dp_p_d_dp_p';
                end
            end
        end

        % Perform refinement of parameters -------------------------------%

        util.verbose_disp('---', 3, calib_config);
        util.verbose_disp(['Refining single parameters with ' optimization_type ' optimization...'], 3, calib_config);

        if calib_config.apply_covariance_optimization
            util.verbose_disp('Applying covariance optimization.', 3, calib_config);

            [A, d, Rs, ts] = obj_calib.refine({A}, ...
                                              {d}, ...
                                              Rs, ...
                                              ts, ...
                                              {eye(3)}, ...
                                              {zeros(3, 1)}, ...
                                              p_cb_ws, ...
                                              p_cb_p_dss, ...
                                              idx_valids, ...
                                              optimization_type, ...
                                              cov_cb_p_dss);
        else
            [A, d, Rs, ts] = obj_calib.refine({A}, ...
                                              {d}, ...
                                              Rs, ...
                                              ts, ...
                                              {eye(3)}, ...
                                              {zeros(3, 1)}, ...
                                              p_cb_ws, ...
                                              p_cb_p_dss, ...
                                              idx_valids, ...
                                              optimization_type);
        end

        % Format outputs
        A = A{1};   % Uncell A
        d = d{1};   % Uncell d

        % Update homographies --------------------------------------------%

        for i = 1:num_boards
            H_w2ps{i} = obj_calib.ARt2H(A, Rs{i}, ts{i});
        end
    end

    % Package outputs ----------------------------------------------------%
    % Config
    calib.config = calib_config;
    % Intrinsics
    calib.cam.intrin.A = A;
    calib.cam.intrin.d = d;
    % Extrinsics
    for i = 1:num_boards
        calib.cam.extrin(i).img_cb = img_cbs(i);
        calib.cam.extrin(i).R = Rs{i};
        calib.cam.extrin(i).t = ts{i};
        calib.cam.extrin(i).p_cb_p_ds = p_cb_p_dss{i};
        calib.cam.extrin(i).cov_cb_p_ds = cov_cb_p_dss{i};
        calib.cam.extrin(i).p_cb_p_d_ms = obj_calib.p_cb_w2p_cb_p_d(p_cb_ws, Rs{i}, ts{i}, A, d);
        calib.cam.extrin(i).idx_valid = idx_valids{i};
        calib.cam.extrin(i).debug = debug{i};
    end
    % Relative extrinsics
    calib.cam.R_1 = eye(3);
    calib.cam.t_1 = zeros(3, 1);
end
