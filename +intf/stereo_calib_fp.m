function calib = stereo_calib_fp(img_cbs, p_fp_p_dss, calib_config, intrin)
    % Performs stereo camera calibration using "four point" method.
    %
    % Inputs:
    %   img_cbs - struct;
    %       .L - class.img; Nx1 calibration board images
    %       .R - class.img; Nx1 calibration board images
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
    %       .debug - struct;

    % Get single four point calibration function
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            f_single_calib_fp = @alg.single_calib_fp_dr;
        otherwise
            error(['Unknown calibration optimization: "' calib_config.calib_optimization '"']);
    end

    % Call stereo four point calibration function
    if exist('intrin', 'var')
        calib = alg.stereo_calib_fp(f_single_calib_fp, img_cbs, p_fp_p_dss, calib_config, intrin);
    else
        calib = alg.stereo_calib_fp(f_single_calib_fp, img_cbs, p_fp_p_dss, calib_config);
    end
end
