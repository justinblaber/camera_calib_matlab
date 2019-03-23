function [p_fp_pss, debug] = multi_fp_detect(img_cbss, calib_config)
    % Obtains the locations of the four points (fiducial markers) around
    % the calibration board images.
    %
    % Inputs:
    %   img_cbss - cell; contains cell of class.img.intf
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %
    % Outputs:
    %   p_fp_pss - cell; contains cell array of four points in pixel
    %       coordinates
    %   debug - struct;

    for i = 1:numel(img_cbss)
        util.verbose_disp('------', 1, calib_config);
        util.verbose_disp(['Performing four-point ' calib_config.fp_detector ' detection for camera ' num2str(i) ' images...'], 1, calib_config);
        [p_fp_pss{i}, debug{i}] = intf.single_fp_detect(img_cbss{i}, calib_config); %#ok<AGROW>
    end
end
