function plot_debug_single_fp_detect_LoG(p_fps, debug_single_fp_detect_LoG, img_cb, calib_config, a)
    % This plots debugging info for four point LoG detection

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end
    cla(a);

    % Get scale factor
    if isnan(calib_config.fp_detect_array_min_size)
        sf = 1;
    else
        size_array = [img_cb.get_height() img_cb.get_width()];
        sf = calib_config.fp_detect_array_min_size/min(size_array);
    end

    % Get array and scale it
    array = img_cb.get_array_gs();
    array = imresize(array, sf);

    % Show image
    imshow(array, [], 'Parent', a)
    hold(a, 'on');

    % Rescale four points and plot
    p_fps = alg.p2imresize(p_fps, sf);
    plot(p_fps(1:2, 1), p_fps(1:2, 2), '-o', 'Color', [0.0000 0.0000 1.0000], ...
         'MarkerSize', 8, 'LineWidth', 2, 'parent', a);
    plot(p_fps(2:3, 1), p_fps(2:3, 2), '-o', 'Color', [1.0000 0.1034 0.7241], ...
         'MarkerSize', 8, 'LineWidth', 2, 'parent', a);
    plot(p_fps(3:4, 1), p_fps(3:4, 2), '-o', 'Color', [1.0000 0.8276 0.0000], ...
         'MarkerSize', 8, 'LineWidth', 2, 'parent', a);

    % Plot blobs and ellipses (does not need to be rescaled)
    axes(a);
    for i = 1:size(debug_single_fp_detect_LoG.blobs, 1)
        external.ellipse(debug_single_fp_detect_LoG.blobs(i, 3), ...
                         debug_single_fp_detect_LoG.blobs(i, 4), ...
                         debug_single_fp_detect_LoG.blobs(i, 5), ...
                         debug_single_fp_detect_LoG.blobs(i, 1), ...
                         debug_single_fp_detect_LoG.blobs(i, 2), ...
                         'r');
    end
    for i = 1:size(debug_single_fp_detect_LoG.ellipses, 1)
        external.ellipse(debug_single_fp_detect_LoG.ellipses(i, 3), ...
                         debug_single_fp_detect_LoG.ellipses(i, 3), ...
                         debug_single_fp_detect_LoG.ellipses(i, 4), ...
                         debug_single_fp_detect_LoG.ellipses(i, 1), ...
                         debug_single_fp_detect_LoG.ellipses(i, 2), ...
                         'g');
    end

    % Remove hold
    hold(a, 'off');
end
