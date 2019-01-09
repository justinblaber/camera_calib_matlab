function plot_patch_match(patch, template, a)
    % This plots a patch and corresponding template

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end
    cla(a);

    % Show patch
    imshow(patch, [], 'Parent', a);

    % Plot boundaries of template over patch
    hold(a, 'on');
    boundaries = bwboundaries(template < (max(template(:)) + min(template(:)))/2);
    for i = 1:numel(boundaries)
        boundary = boundaries{i};
        p = plot(a, boundary(:, 2), boundary(:, 1), 'Color', 'g', 'LineWidth', 1);
        p.Color(4) = 0.75; % transparency
    end

    % Remove hold
    hold(a, 'off');
end
