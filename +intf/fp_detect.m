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
    %   debug - cell;

    % Get four point detector function
    switch calib_config.fp_detector
        case 'LoG'
            f_fp_detect = @alg.fp_detect_LoG;
        otherwise
            error(['Unknown four point detector: "' calib_config.fp_detector '"']);
    end

    % Get scale factor
    if isnan(calib_config.fp_detect_array_min_size)
        sf = 1;
    else
        sf = calib_config.fp_detect_array_min_size/min(img_cbs(1).get_size());
    end

    % Get number of cams and boards
    num_cams = size(img_cbs, 2);
    num_boards = size(img_cbs, 1);

    p_fp_pss = cell(num_boards, num_cams);
    debug = cell(num_boards, num_cams);
    for i = 1:num_cams
        util.verbose_disp('------', 1, calib_config);
        util.verbose_disp(['Performing four-point ' calib_config.fp_detector ' detection for camera ' num2str(i) ' images...'], 1, calib_config);
        util.verbose_disp('---', 1, calib_config);

        % Cycle over images and get four points
        for j = 1:num_boards
            util.verbose_fprintf('Performing four-point %s detection for image: %s. ', ...
                                 calib_config.fp_detector, ...
                                 img_cbs(j, i).get_name(), ...
                                 1, ...
                                 calib_config);

            % Get array and scale it
            array = img_cbs(j, i).get_array_gs();
            array = imresize(array, sf);

            % Get four points
            t = tic;
            [p_fp_pss{j, i}, debug{j, i}] = f_fp_detect(array, calib_config);
            time = toc(t);

            % Scale four points
            p_fp_pss{j, i} = alg.imresize2p(p_fp_pss{j, i}, sf);

            util.verbose_fprintf(['Time ellapsed: %f seconds.' newline], time, 1, calib_config);
        end
    end
end
