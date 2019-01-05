function plot_patch_match(patch_match, num, a)
    % This plots a "patch match"

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end
    cla(a);

    % Show patch
    imshow(patch_match.patch, [], 'Parent', a);
    title(a, [num2str(num) ' (CC val: ' num2str(patch_match.val_cc) ')'], 'FontSize', 7);

    % Plot boundaries of template over patch
    hold(a, 'on');
    boundaries = bwboundaries(patch_match.template < (max(patch_match.template(:)) + min(patch_match.template(:)))/2);
    for i = 1:numel(boundaries)
        boundary = boundaries{i};
        p = plot(a, boundary(:, 2), boundary(:, 1), 'Color', 'g', 'LineWidth', 1);
        p.Color(4) = 0.75; % transparency
    end

    % Remove hold
    hold(a, 'off');
end
