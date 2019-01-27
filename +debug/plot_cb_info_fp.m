function plot_cb_info_fp(p_fp_ws, p_cb_ws, boundary_ws, height_cb, width_cb, a)
    % This will plot calibration board info

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end
    cla(a);

    % Format axes
    axis(a, 'equal');
    set(a, 'Ydir', 'reverse', 'Xlim', [0 width_cb], 'Ylim', [0 height_cb]);
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

    % Plot boundaries
    for i = 1:numel(boundary_ws)
        plot(vertcat(boundary_ws{i}(:,1), boundary_ws{i}(1,1)), ...
             vertcat(boundary_ws{i}(:,2), boundary_ws{i}(1,2)), ...
             ':r', 'parent', a);
    end
     
    % Remove hold
    hold(a, 'off');
end
