function plot_cb_geom(obj_cb_geom, a)
    % This will plot calibration board class

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    % Format axes
    axis(a, 'equal');
    hold(a, 'on');

    if isa(obj_cb_geom, 'class.cb_geom.size_intf')
        set(a, 'Ydir', 'reverse', 'Xlim', [0 obj_cb_geom.get_cb_width()], ...
                                  'Ylim', [0 obj_cb_geom.get_cb_height()]);
    end

    if isa(obj_cb_geom, 'class.cb_geom.target_intf')
        p_cb_ws = obj_cb_geom.get_p_cb_ws();
        boundary_ws = obj_cb_geom.get_p_cb_w_boundaries();

        % Plot board points
        plot(p_cb_ws(:, 1), p_cb_ws(:, 2), 'gs', 'MarkerSize', 8, ...
             'MarkerFaceColor', 'w', 'parent', a);
        text(p_cb_ws(:, 1), p_cb_ws(:, 2), strtrim(cellstr(num2str([1:size(p_cb_ws, 1)]'))), ...
             'FontSize', 5, 'HorizontalAlignment', 'center', 'color', 'k', 'parent', a); %#ok<NBRAK>

        % Plot boundaries
        for i = 1:numel(boundary_ws)
            plot(vertcat(boundary_ws{i}(:, 1), boundary_ws{i}(1, 1)), ...
                 vertcat(boundary_ws{i}(:, 2), boundary_ws{i}(1, 2)), ...
                 ':r', 'parent', a);
        end
    end

    if isa(obj_cb_geom, 'class.cb_geom.fp_intf')
        p_fp_ws = obj_cb_geom.get_p_fp_ws();

        % Plot four points
        plot(p_fp_ws(:, 1), p_fp_ws(:, 2), 'bo', 'MarkerSize', 8, ...
             'MarkerFaceColor', 'w', 'LineWidth', 1.5, 'parent', a);
        text(p_fp_ws(:, 1), p_fp_ws(:, 2), cellstr(num2str([1:4]')), ...
             'FontSize', 5, 'HorizontalAlignment', 'center', 'parent', a); %#ok<NBRAK>
    end

    % Remove hold
    hold(a, 'off');
end
