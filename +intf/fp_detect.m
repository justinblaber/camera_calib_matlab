function [p_fp_pss, debug] = fp_detect(img_cbs, calib_config)
    % Obtains the locations of the four points (fiducial markers) around
    % the calibration board images.
    %
    % Inputs:
    %   img_cbs - class.img.intf; MxN class.img.intf
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %
    % Outputs:
    %   p_fp_pss - cell; MxN cell array of four points in pixel coordinates
    %   debug - struct;

    % Get number of cams
    num_cams = size(img_cbs, 2);
    
    for i = 1:num_cams
        util.verbose_disp('------', 1, calib_config);
        util.verbose_disp(['Performing four-point ' calib_config.fp_detector ' detection for camera ' num2str(i) ' images...'], 1, calib_config);
        [p_fp_pss(:, i), debug(:, i)] = intf.single_fp_detect(img_cbs(:, i), calib_config); %#ok<AGROW>
    end
end
