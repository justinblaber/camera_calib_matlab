function calib = single_calib_fp(img_cbs, p_fp_p_dss, calib_config, intrin)
    % Performs camera calibration using "four point distortion refinement"
    % method.
    %
    % Inputs:
    %   img_cbs - class.img; Nx1 calibration board images
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

    % Get single four point calibration function
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            f_single_calib_fp = @alg.single_calib_fp_dr;
        otherwise
            error(['Unknown calibration optimization: "' opts.calib_optimization '"']);
    end
    
    % Call single four point calibration function
    if exist('intrin', 'var')
        calib = f_single_calib_fp(img_cbs, p_fp_p_dss, calib_config, intrin);
    else
        calib = f_single_calib_fp(img_cbs, p_fp_p_dss, calib_config);
    end
end
