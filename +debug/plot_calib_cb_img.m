function plot_calib_cb_img(extrin, intrin, a)
    % This will plot calibration results over a calibration board image

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    % Show image
    array = extrin.img_cb.get_array_gs();
    imshow(array, [], 'Parent', a);
    hold(a, 'on');

    % Plot four points
    if isfield(extrin, 'p_fp_p_ds')
        p_fp_p_ds = extrin.p_fp_p_ds;
        plot(p_fp_p_ds(:, 1), p_fp_p_ds(:, 2), 'bo', 'MarkerSize', 8, ...
             'MarkerFaceColor', 'w', 'LineWidth', 1.5, 'parent', a);
        text(p_fp_p_ds(:, 1), p_fp_p_ds(:, 2), strtrim(cellstr(num2str([1:4]'))), ...
             'FontSize', 5, 'HorizontalAlignment', 'center', 'parent', a); %#ok<NBRAK>
    end

    % Plot model distorted pixel points
    p_cb_p_d_ms = extrin.p_cb_p_d_ms(extrin.idx_valid, :);
    plot(p_cb_p_d_ms(:, 1), p_cb_p_d_ms(:, 2), 'r+', 'MarkerSize', 6, 'LineWidth', 1, 'parent', a);

    % Plot measured distorted pixel points
    p_cb_p_ds = extrin.p_cb_p_ds(extrin.idx_valid, :);
    plot(p_cb_p_ds(:, 1), p_cb_p_ds(:, 2), 'gs', 'MarkerSize', 6, 'LineWidth', 1, 'parent', a);

    % Plot displacements
    quiver(p_cb_p_ds(:, 1), p_cb_p_ds(:, 2), ...
           p_cb_p_d_ms(:, 1)-p_cb_p_ds(:, 1), ...
           p_cb_p_d_ms(:, 2)-p_cb_p_ds(:, 2), ...
           'color', 'b', 'LineWidth', 1, 'AutoScale', 'off', 'parent', a);

    % Plot principle point
    A = intrin.A;
    plot([A(1, 3) A(1, 3)], [1 size(array, 1)], '--g', 'parent', a);
    plot([1 size(array, 2)], [A(2, 3) A(2, 3)], '--g', 'parent', a);

    % Plot center of image (to compare principle point)
    plot([(size(array, 2)+1)/2 (size(array, 2)+1)/2], [1 size(array, 1)], '--r', 'parent', a);
    plot([1 size(array, 2)], [(size(array, 1)+1)/2 (size(array, 1)+1)/2], '--r', 'parent', a);

    % Remove hold
    hold(a, 'off');
end
