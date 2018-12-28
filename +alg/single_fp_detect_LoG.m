function [p_fp_pss, debugs] = single_fp_detect_LoG(img_cbs, calib_config)
    % Obtains the locations of the four points (fiducial markers) around
    % the calibration board images.
    %
    % Inputs:
    %   img_cbs - util.img; Nx1 calibration board images
    %   calib_config - struct; struct returned by util.read_calib_config()
    %
    % Outputs:
    %   p_fp_pss - cell; Mx1 cell array of four points in pixel coordinates
    %   debugs - struct; used for debugging purposes

    util.verbose_disp('---', 1, calib_config);

    % Get scale factor
    if calib_config.fp_detect_array_min_size == 0
        sf = 1;
    else
        size_array = [img_cbs(1).get_height() img_cbs(1).get_width()];
        sf = calib_config.fp_detect_array_min_size/min(size_array);
    end

    % Cycle over images and get four points
    for i = 1:numel(img_cbs)
        util.verbose_fprintf('Performing four-point detection for image: %s. ', img_cbs(i).get_path(), 1, calib_config);

        % Get array and scale it
        array = img_cbs(i).get_array_gs();
        array = imresize(array, sf);

        % Get four points
        t = tic;
        [p_fp_pss{i}, debugs(i)] = alg.fp_detect_LoG(array, calib_config); %#ok<AGROW>
        time = toc(t);

        % Scale four points
        p_fp_pss{i} = alg.imresize2p(p_fp_pss{i}, sf); %#ok<AGROW>

        util.verbose_fprintf(['Time ellapsed: %f seconds.' newline], time, 1, calib_config);
    end
end
