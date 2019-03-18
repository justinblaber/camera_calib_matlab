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

    % Get single four point calibration function
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            f_single_calib_fp = @alg.single_calib_fp_dr;
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

    % Call single four point calibration function
    if exist('intrin', 'var')
        calib = f_single_calib_fp(calib_config.obj_A, ...
                                  calib_config.obj_R, ...
                                  obj_cb_w2p, ...
                                  obj_distortion, ...
                                  calib_config.obj_cb_geom, ...
                                  img_cbs, ...
                                  p_fp_p_dss, ...
                                  calib_config, ...
                                  intrin);
    else
        calib = f_single_calib_fp(calib_config.obj_A, ...
                                  calib_config.obj_R, ...
                                  obj_cb_w2p, ...
                                  obj_distortion, ...
                                  calib_config.obj_cb_geom, ...
                                  img_cbs, ...
                                  p_fp_p_dss, ...
                                  calib_config);
    end
end
