function calib = multi_calib_H(f_single_calib_H, obj_A, obj_R, obj_cb_w2p, obj_distortion, obj_cb_geom, img_cbss, H_w2pss, calib_config, intrins)

    util.verbose_disp('------------', 1, calib_config);
    util.verbose_disp('Performing stereo calibration with homography method...', 1, calib_config);

    % Get number of cameras and number of boards    
    num_cams = numel(img_cbss);
    num_boards = numel(img_cbss{1});
    
    % Perform single calibrations ----------------------------------------%
    
    for i = 1:num_cams
        % Calibrate left camera
        util.verbose_disp('---------', 1, calib_config);
        util.verbose_disp(['Calibrating camera ' num2str(i) '...'], 1, calib_config);

        if exist('intrins', 'var')
            calib_cam{i} = f_single_calib_H(obj_A, ...
                                            obj_R, ...
                                            obj_cb_w2p, ...
                                            obj_distortion, ...
                                            obj_cb_geom, ...
                                            img_cbss{i}, ...
                                            H_w2pss{i}, ...
                                            calib_config, ...
                                            intrins{i}); %#ok<AGROW>
        else
            calib_cam{i} = f_single_calib_H(obj_A, ...
                                            obj_R, ...
                                            obj_cb_w2p, ...
                                            obj_distortion, ...
                                            obj_cb_geom, ...
                                            img_cbss{i}, ...
                                            H_w2pss{i}, ...
                                            calib_config); %#ok<AGROW>
        end

        % Remove config since its redundant
        calib_cam{i} = rmfield(calib_cam{i}, 'config'); %#ok<AGROW>
    end
   
    % Package initial guesses and other parameters -----------------------%

    for i = 1:num_cams
        % Get intrinsics
        As{i} = calib_cam{i}.intrin.A; %#ok<AGROW>
        ds{i} = calib_cam{i}.intrin.d; %#ok<AGROW>

        % Get extrinsics
        Rss{i} = {calib_cam{i}.extrin.R}; %#ok<AGROW>
        tss{i} = {calib_cam{i}.extrin.t}; %#ok<AGROW>
        p_cb_p_dsss{i} = {calib_cam{i}.extrin.p_cb_p_ds}; %#ok<AGROW>
        cov_cb_p_dsss{i} = {calib_cam{i}.extrin.cov_cb_p_ds}; %#ok<AGROW>
        idx_validss{i} = {calib_cam{i}.extrin.idx_valid}; %#ok<AGROW>
    end

    % Perform multi calibration ------------------------------------------%

    % Get calibration object
    obj_multi_calib = class.calib.multi(obj_A, ...
                                        obj_R, ...
                                        obj_cb_w2p, ...
                                        obj_distortion, ...
                                        calib_config);

    % Get the calibration board points in world coordinates
    p_cb_ws = obj_cb_geom.get_p_cb_ws();

    % Get optimization type
    if exist('intrin', 'var')
        optimization_type = 'extrinsic'; % Only optimize extrinsics
    else
        optimization_type = 'full';      % Optimize intrinsics and extrinsics
    end

    % Get initial guesses for transform between camera 1 and camera i ----%

    for i = 1:num_cams    
        % Get least squares linear initial guess for R_s
        r = [];
        R = [];
        for j = 1:num_boards
            r = vertcat(r, Rss{i}{j}(:)); %#ok<AGROW>
            R = vertcat(R, [Rss{1}{j}(1, 1)*eye(3) Rss{1}{j}(2, 1)*eye(3) Rss{1}{j}(3, 1)*eye(3);
                            Rss{1}{j}(1, 2)*eye(3) Rss{1}{j}(2, 2)*eye(3) Rss{1}{j}(3, 2)*eye(3);
                            Rss{1}{j}(1, 3)*eye(3) Rss{1}{j}(2, 3)*eye(3) Rss{1}{j}(3, 3)*eye(3)]); %#ok<AGROW>
        end

        % Get least squares approximation
        R_1s{i} = reshape(alg.safe_lscov(R, r), 3, 3); %#ok<AGROW>
        R_1s{i} = alg.approx_R(R_1s{i}); %#ok<AGROW>                % Get best rotational approximation

        % Get least squares linear guess for t_s
        t = [];
        T = [];
        for j = 1:num_boards
            t = vertcat(t, tss{i}{j}-Rss{i}{j}*Rss{1}{j}'*tss{1}{j}); %#ok<AGROW>
            T = vertcat(T, eye(3)); %#ok<AGROW>
        end

        % Get least squares approximation
        t_1s{i} = alg.safe_lscov(T, t); %#ok<AGROW>
    end
    
    % NOTE: R_1s{1} should be identity and t_1s{1} should be zero vector.
    
    % Perform nonlinear refinement of all parameters ---------------------%

    util.verbose_disp('---------', 1, calib_config);
    util.verbose_disp(['Refining multi parameters with ' optimization_type ' optimization...'], 1, calib_config);

    if calib_config.apply_covariance_optimization
        util.verbose_disp('Applying covariance optimization.', 3, calib_config);

        [As, ds, Rss, tss, R_1s, t_1s] = obj_multi_calib.refine(As, ...
                                                                ds, ...
                                                                Rss, ...
                                                                tss, ...
                                                                R_1s, ...
                                                                t_1s, ...
                                                                p_cb_ws, ...
                                                                p_cb_p_dsss, ...
                                                                idx_validss, ...
                                                                optimization_type, ...
                                                                cov_cb_p_dsss);
    else
        [As, ds, Rss, tss, R_1s, t_1s] = obj_multi_calib.refine(As, ...
                                                                ds, ...
                                                                Rss, ...
                                                                tss, ...
                                                                R_1s, ...
                                                                t_1s, ...
                                                                p_cb_ws, ...
                                                                p_cb_p_dsss, ...
                                                                idx_validss, ...
                                                                optimization_type);
    end

    % Repackage outputs --------------------------------------------------%
    % Config
    calib.config = calib_config;
    calib.cam = [calib_cam{:}];
    for i = 1:num_cams
        % Overwrite intrinsics and extrinsics with stereo optimized values
        calib.cam(i).intrin.A = As{i};
        calib.cam(i).intrin.d = ds{i};
        for j = 1:num_boards
            calib.cam(i).extrin(j).R = Rss{i}{j};
            calib.cam(i).extrin(j).t = tss{i}{j};
            calib.cam(i).extrin(j).p_cb_p_d_ms = obj_multi_calib.p_cb_w2p_cb_p_d(p_cb_ws, Rss{i}{j}, tss{i}{j}, As{i}, ds{i});
        end
        % Store relative extrinsics
        calib.cam(i).R_1 = R_1s{i};
        calib.cam(i).t_1 = t_1s{i};
    end
end
