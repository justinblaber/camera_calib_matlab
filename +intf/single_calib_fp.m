function calib = single_calib_fp(img_cbs, p_fp_p_dss, calib_config, intrin)
    % Performs camera calibration using "four point" method.
    %
    % Inputs:
    %   img_cbs - class.img.intf; Nx1 calibration board images
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
    %           .img_cb - class.img.intf; calibration board image
    %           .R - array; 3x3 rotation matrix
    %           .t - array; 3x1 translation vector
    %           .p_cb_p_ds - array; calibration board distorted pixel
    %               points
    %           .cov_cb_p_ds - cell; covariances of calibration board
    %               distorted pixel points
    %           .p_cb_p_d_ms - array; calibration board model distorted
    %               pixel points
    %           .idx_valid - array; valid calibration board points
    %           .p_fp_p_ds - array; four point box around the calibration
    %               board image in distorted pixel coordinates

    % Get single four point calibration function
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            f_single_calib_H = @alg.single_calib_H_dr;
        otherwise
            error(['Unknown calibration optimization: "' calib_config.calib_optimization '"']);
    end

    % Get world to pixel stuff
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            switch calib_config.target
                case 'checker'
                    obj_cb_w2p = class.calib.cb_w2p_p2p(calib_config);
                case 'circle'
                    obj_cb_w2p = class.calib.cb_w2p_c2e(calib_config);
                otherwise
                    error(['Cannot obtain world to pixel parameterization ' ...
                           'for ' calib_config.calib_optimization ' ' ...
                           'optimization with ' calib_config.target ' ' ...
                           'target.']);
            end
        otherwise
            error(['Cannot obtain world to pixel parameterization for ' ...
                   calib_config.calib_optimization ' optimization.']);
    end

    % Get distortion object
    obj_distortion = class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config);

    % Get initial guess for homographies ---------------------------------%

    % Get the four point boxes in world coordinates
    p_fp_ws = calib_config.obj_cb_geom.get_p_fp_ws();

    % Get number of boards
    num_boards = numel(img_cbs);

    for i = 1:num_boards
        % Get four point box in pixel coordinates
        if exist('intrin', 'var')
            % If intrinsics are passed in, undistort four point box
            p_fp_ps = obj_distortion.p_p_d2p_p(p_fp_p_dss{i}, ...
                                               p_fp_p_dss{i}, ...     % Use distorted points for initial guess
                                               intrin.A, ...
                                               intrin.d);
        else
            % If intrinsics arent available, assume distortion is small
            p_fp_ps = p_fp_p_dss{i};
        end

        % Compute homography - use direct p2p method
        H_w2ps{i} = alg.homography_p2p(p_fp_ws, ...
                                       p_fp_ps, ...
                                       calib_config); %#ok<AGROW>
    end

    % Call single homography calibration function ------------------------%

    if exist('intrin', 'var')
        calib = f_single_calib_H(calib_config.obj_A, ...
                                 calib_config.obj_R, ...
                                 obj_cb_w2p, ...
                                 obj_distortion, ...
                                 calib_config.obj_cb_geom, ...
                                 img_cbs, ...
                                 H_w2ps, ...
                                 calib_config, ...
                                 intrin);
    else
        calib = f_single_calib_H(calib_config.obj_A, ...
                                 calib_config.obj_R, ...
                                 obj_cb_w2p, ...
                                 obj_distortion, ...
                                 calib_config.obj_cb_geom, ...
                                 img_cbs, ...
                                 H_w2ps, ...
                                 calib_config);
    end

    % Append four point box to output ------------------------------------%

    for i = 1:num_boards
        calib.extrin(i).p_fp_p_ds = p_fp_p_dss{i};
    end
end
