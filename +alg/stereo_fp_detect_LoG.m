function [p_fp_pss, debug] = stereo_fp_detect_LoG(img_cbs, calib_config)
    % Obtains the locations of the four points (fiducial markers) around
    % the calibration board images.
    %
    % Inputs:
    %   img_cbs - struct; contains:
    %       .L and .R - class.img; Nx1 calibration board images
    %   calib_config - struct; struct returned by util.load_calib_config()
    %
    % Outputs:
    %   p_fp_pss - struct; contains:
    %       .L and .R - cell; Mx1 cell array of four points in pixel
    %           coordinates
    %   debug - struct;

    % Do left images
    util.verbose_disp('------', 1, calib_config);
    util.verbose_disp('Performing four-point detection for left images...', 1, calib_config);
    [p_fp_pss.L, debug.L] = alg.single_fp_detect_LoG(img_cbs.L, calib_config);

    % Do right images
    util.verbose_disp('------', 1, calib_config);
    util.verbose_disp('Performing four-point detection for right images...', 1, calib_config);
    [p_fp_pss.R, debug.R] = alg.single_fp_detect_LoG(img_cbs.R, calib_config);
end
