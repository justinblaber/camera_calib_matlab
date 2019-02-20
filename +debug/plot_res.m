function plot_res(ress, colors, alphas, lim, a)
    % This plots residuals

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    % Hold
    hold(a, 'on');

    % Plot residuals; plot highest alpha last
    [~, idx_sorted] = sort(alphas);
    for i = 1:numel(ress)
        idx = idx_sorted(i);
        res = ress{idx};
        scatter(res(:, 1), res(:, 2), 12, ...
                'MarkerFaceColor', colors(idx, :), 'MarkerFaceAlpha', alphas(idx), ...
                'MarkerEdgeAlpha', 0, 'parent', a);
    end

    % Plot dashed line to indicate zero
    plot([0 0], [-lim lim], '--r', 'parent', a);
    plot([-lim lim], [0 0], '--r', 'parent', a);

    % Format plot
    set(a, 'xlim', [-lim lim], 'ylim', [-lim lim]);
    daspect(a, [1 1 1]);

    % Remove hold
    hold(a, 'off');
end
