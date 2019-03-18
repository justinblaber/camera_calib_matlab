function calib = stereo_calib_fp(img_cbs, p_fp_p_dss, calib_config, intrin)
    % Performs stereo camera calibration using "four point" method.
    %
    % Inputs:
    %   img_cbs - struct;
    %       .L - class.img.intf; Nx1 calibration board images
    %       .R - class.img.intf; Nx1 calibration board images
    %   p_fp_p_dss - struct;
    %       .L - cell; Nx1 cell of four point boxes around the
    %           calibration board images in distorted pixel coordinates
    %       .R - cell; Nx1 cell of four point boxes around the
    %           calibration board images in distorted pixel coordinates
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %   intrin - struct; optional. If passed in, intrinsics will not be
    %       optimized.
    %       .L - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %       .R - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %
    % Outputs:
    %   calib - struct;
    %       .config - struct; copy of input calib_config
    %       .L - struct; calibration for left camera
    %       .R - struct; calibration for right camera
    %       .R_s - array; 3x3 rotation matrix describing rotation from
    %           left to right camera
    %       .t_s - array; 3x1 translation vector describing translation
    %           from left to right camera

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
    num_boards = numel(img_cbs.L);

    for i = 1:num_boards
        % Get four point box in pixel coordinates
        if exist('intrin', 'var')
            % If intrinsics are passed in, undistort four point box
            p_fp_ps.L = obj_distortion.p_p_d2p_p(p_fp_p_dss.L{i}, ...
                                                 p_fp_p_dss.L{i}, ...     % Use distorted points for initial guess
                                                 intrin.A, ...
                                                 intrin.d);
            p_fp_ps.R = obj_distortion.p_p_d2p_p(p_fp_p_dss.R{i}, ...
                                                 p_fp_p_dss.R{i}, ...     % Use distorted points for initial guess
                                                 intrin.A, ...
                                                 intrin.d);
        else
            % If intrinsics arent available, assume distortion is small
            p_fp_ps.L = p_fp_p_dss.L{i};
            p_fp_ps.R = p_fp_p_dss.R{i};
        end

        % Compute homography - use direct p2p method
        H_w2ps.L{i} = alg.homography_p2p(p_fp_ws, ...
                                         p_fp_ps.L, ...
                                         calib_config);
        H_w2ps.R{i} = alg.homography_p2p(p_fp_ws, ...
                                         p_fp_ps.R, ...
                                         calib_config);
    end

    % Call stereo homography calibration function ------------------------%

    if exist('intrin', 'var')
        calib = alg.stereo_calib_H(f_single_calib_H, ...
                                   calib_config.obj_A, ...
                                   calib_config.obj_R, ...
                                   obj_cb_w2p, ...
                                   obj_distortion, ...
                                   calib_config.obj_cb_geom, ...
                                   img_cbs, ...
                                   H_w2ps, ...
                                   calib_config, ...
                                   intrin);
    else
        calib = alg.stereo_calib_H(f_single_calib_H, ...
                                   calib_config.obj_A, ...
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
        calib.L.extrin(i).p_fp_p_ds = p_fp_p_dss.L{i};
        calib.R.extrin(i).p_fp_p_ds = p_fp_p_dss.R{i};
    end
end
