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

    % Call stereo four point calibration function
    if exist('intrin', 'var')
        calib = alg.stereo_calib_fp(f_single_calib_fp, ...
                                    calib_config.obj_A, ...
                                    calib_config.obj_R, ...
                                    obj_cb_w2p, ...
                                    obj_distortion, ...
                                    calib_config.obj_cb_geom, ...
                                    img_cbs, ...
                                    p_fp_p_dss, ...
                                    calib_config, ...
                                    intrin);
    else
        calib = alg.stereo_calib_fp(f_single_calib_fp, ...
                                    calib_config.obj_A, ...
                                    calib_config.obj_R, ...
                                    obj_cb_w2p, ...
                                    obj_distortion, ...
                                    calib_config.obj_cb_geom, ...
                                    img_cbs, ...
                                    p_fp_p_dss, ...
                                    calib_config);
    end
end
