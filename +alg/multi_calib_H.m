function calib = multi_calib_H(f_single_calib_H, obj_A, obj_R, obj_cb_w2p, obj_distortion, obj_cb_geom, img_cbs, H_w2ps, calib_config, intrins)

    util.verbose_disp('------------', 1, calib_config);
    util.verbose_disp('Performing multi calibration with homography method...', 1, calib_config);
    
    % Perform single calibrations ----------------------------------------%
    
    % Get number of cameras and boards
    num_cams = size(img_cbs, 2);
    num_boards = size(img_cbs, 1);
    
    for i = 1:num_cams
        % Calibrate camera
        util.verbose_disp('---------', 1, calib_config);
        util.verbose_disp(['Calibrating camera ' num2str(i) '...'], 1, calib_config);

        if exist('intrins', 'var')
            calib_cam = f_single_calib_H(obj_A, ...
                                         obj_R, ...
                                         obj_cb_w2p, ...
                                         obj_distortion, ...
                                         obj_cb_geom, ...
                                         img_cbs(:, i), ...
                                         H_w2ps(:, i), ...
                                         calib_config, ...
                                         intrins(i));
        else
            calib_cam = f_single_calib_H(obj_A, ...
                                         obj_R, ...
                                         obj_cb_w2p, ...
                                         obj_distortion, ...
                                         obj_cb_geom, ...
                                         img_cbs(:, i), ...
                                         H_w2ps(:, i), ...
                                         calib_config);
        end

        % Remove config since its redundant
        calib_cam = rmfield(calib_cam, 'config');
        
        % Append
        calib_cams(i) = calib_cam; %#ok<AGROW>
    end
   
    % Package initial guesses and other parameters -----------------------%

    for i = 1:num_cams
        % Get intrinsics
        As{i} = calib_cams(i).intrin.A; %#ok<AGROW>
        ds{i} = calib_cams(i).intrin.d; %#ok<AGROW>

        % Get extrinsics
        for j = 1:num_boards
            Rs{j, i} = calib_cams(i).extrin(j).R; %#ok<AGROW>
            ts{j, i} = calib_cams(i).extrin(j).t; %#ok<AGROW>
            p_cb_p_dss{j, i} = calib_cams(i).extrin(j).p_cb_p_ds; %#ok<AGROW>
            cov_cb_p_dss{j, i} = calib_cams(i).extrin(j).cov_cb_p_ds; %#ok<AGROW>
            idx_valids{j, i} = calib_cams(i).extrin(j).idx_valid; %#ok<AGROW>
        end
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
            r = vertcat(r, Rs{j, i}(:)); %#ok<AGROW>
            R = vertcat(R, [Rs{j, 1}(1, 1)*eye(3) Rs{j, 1}(2, 1)*eye(3) Rs{j, 1}(3, 1)*eye(3);
                            Rs{j, 1}(1, 2)*eye(3) Rs{j, 1}(2, 2)*eye(3) Rs{j, 1}(3, 2)*eye(3);
                            Rs{j, 1}(1, 3)*eye(3) Rs{j, 1}(2, 3)*eye(3) Rs{j, 1}(3, 3)*eye(3)]); %#ok<AGROW>
        end

        % Get least squares approximation
        R_1s{i} = reshape(alg.safe_lscov(R, r), 3, 3); %#ok<AGROW>
        R_1s{i} = alg.approx_R(R_1s{i}); %#ok<AGROW>                % Get best rotational approximation

        % Get least squares linear guess for t_s
        t = [];
        T = [];
        for j = 1:num_boards
            t = vertcat(t, ts{j, i}-Rs{j, i}*Rs{j, 1}'*ts{j, 1}); %#ok<AGROW>
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

        [As, ds, Rs, ts, R_1s, t_1s] = obj_multi_calib.refine(As, ...
                                                              ds, ...
                                                              Rs, ...
                                                              ts, ...
                                                              R_1s, ...
                                                              t_1s, ...
                                                              p_cb_ws, ...
                                                              p_cb_p_dss, ...
                                                              idx_valids, ...
                                                              optimization_type, ...
                                                              cov_cb_p_dss);
    else
        [As, ds, Rs, ts, R_1s, t_1s] = obj_multi_calib.refine(As, ...
                                                              ds, ...
                                                              Rs, ...
                                                              ts, ...
                                                              R_1s, ...
                                                              t_1s, ...
                                                              p_cb_ws, ...
                                                              p_cb_p_dss, ...
                                                              idx_valids, ...
                                                              optimization_type);
    end

    % Repackage outputs --------------------------------------------------%
    % Config
    calib.config = calib_config;    
    % Store individual calibrations
    for i = 1:num_cams
        calib.cam(i) = calib_cams(i);
    end
    % Overwrite stuff
    for i = 1:num_cams
        % Overwrite intrinsics and extrinsics with stereo optimized values
        calib.cam(i).intrin.A = As{i};
        calib.cam(i).intrin.d = ds{i};
        for j = 1:num_boards
            calib.cam(i).extrin(j).R = Rs{j, i};
            calib.cam(i).extrin(j).t = ts{j, i};
            calib.cam(i).extrin(j).p_cb_p_d_ms = obj_multi_calib.p_cb_w2p_cb_p_d(p_cb_ws, Rs{j, i}, ts{j, i}, As{i}, ds{i});
        end
        % Store relative extrinsics
        calib.cam(i).R_1 = R_1s{i};
        calib.cam(i).t_1 = t_1s{i};
    end
end
