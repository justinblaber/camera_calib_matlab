function [p_fp_pss, debug] = single_fp_detect_LoG(img_cbs, calib_config)
    % Obtains the locations of the four points (fiducial markers) around
    % the calibration board images.
    %
    % Inputs:
    %   img_cbs - class.img; Nx1 calibration board images
    %   calib_config - struct; struct returned by util.load_calib_config()
    %
    % Outputs:
    %   p_fp_pss - cell; Mx1 cell array of four points in pixel coordinates
    %   debug - struct;

    util.verbose_disp('---', 1, calib_config);

    % Get scale factor
    if isnan(calib_config.fp_detect_array_min_size)
        sf = 1;
    else
        sf = calib_config.fp_detect_array_min_size/min([img_cbs(1).get_height() img_cbs(1).get_width()]);
    end

    % Cycle over images and get four points
    for i = 1:numel(img_cbs)
        util.verbose_fprintf('Performing four-point detection for image: %s. ', img_cbs(i).get_path(), 1, calib_config);

        % Get array and scale it
        array = img_cbs(i).get_array_gs();
        array = imresize(array, sf);

        % Get four points
        t = tic;
        [p_fp_pss{i}, debug(i)] = alg.fp_detect_LoG(array, calib_config); %#ok<AGROW>
        time = toc(t);

        % Scale four points
        p_fp_pss{i} = alg.imresize2p(p_fp_pss{i}, sf); %#ok<AGROW>

        util.verbose_fprintf(['Time ellapsed: %f seconds.' newline], time, 1, calib_config);
    end
end
