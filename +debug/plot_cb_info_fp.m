function plot_cb_info_fp(opts, a)
    % This will plot calibration board info

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end
    cla(a);

    % Get calibration world points and four points in world coordinates
    p_cb_ws = alg.p_cb_w(opts);
    p_fp_ws = alg.p_fp_w(opts);

    % Format axes
    padding = opts.target_spacing/2;
    bb_axes_w = [min([p_cb_ws; p_fp_ws]) - padding; ...
                 max([p_cb_ws; p_fp_ws]) + padding];
    axis(a, 'equal');
    set(a, 'Ydir', 'reverse', 'Xlim', bb_axes_w(:, 1), 'Ylim', bb_axes_w(:, 2));
    hold(a, 'on');

    % Plot four points
    plot(p_fp_ws(:, 1), p_fp_ws(:, 2), 'bo', 'MarkerSize', 8, ...
         'MarkerFaceColor', 'w', 'LineWidth', 1.5, 'parent', a);
    text(p_fp_ws(:, 1), p_fp_ws(:, 2), cellstr(num2str([1:4]')), ...
         'FontSize', 5, 'HorizontalAlignment', 'center', 'parent', a); %#ok<NBRAK>

    % Plot board points
    plot(p_cb_ws(:, 1), p_cb_ws(:, 2), 'gs', 'MarkerSize', 8, ...
         'MarkerFaceColor', 'w', 'parent', a);
    text(p_cb_ws(:, 1), p_cb_ws(:, 2), strtrim(cellstr(num2str([1:size(p_cb_ws, 1)]'))), ...
         'FontSize', 5, 'HorizontalAlignment', 'center', 'color', 'k', 'parent', a); %#ok<NBRAK>

    % Remove hold
    hold(a, 'off');
end
