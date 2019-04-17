function plot_fp(array, p_fps, a)
    % This plots debugging info for four point detection

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    % Show image
    imshow(array, [], 'Parent', a)
    hold(a, 'on');

    % Plot four points
    plot(p_fps(1:2, 1), p_fps(1:2, 2), '-o', 'Color', [0.0000 0.0000 1.0000], ...
         'MarkerSize', 8, 'LineWidth', 2, 'parent', a);
    plot(p_fps(2:3, 1), p_fps(2:3, 2), '-o', 'Color', [1.0000 0.1034 0.7241], ...
         'MarkerSize', 8, 'LineWidth', 2, 'parent', a);
    plot(p_fps(3:4, 1), p_fps(3:4, 2), '-o', 'Color', [1.0000 0.8276 0.0000], ...
         'MarkerSize', 8, 'LineWidth', 2, 'parent', a);

    % Remove hold
    hold(a, 'off');
end
