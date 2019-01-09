function plot_cb_3D(xform_w2s, color, alpha, opts, a)
    % This will plot a calibration board in 3D given an affine transform

    % Matlab's 3D plot is not very good; to get it in the orientation I want,
    % I've just switched the x, y, and z components with:
    %   x => y
    %   y => z
    %   z => x

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    % Get world coordinates
    p_cb_ws = alg.p_cb_w(opts);

    % Get bounding box
    bb_cb_w = [min(p_cb_ws); ...
               max(p_cb_ws)];

    % Form box
    % Note:
    %    p1 - p3
    %     |    |
    %    p2 - p4
    box_w = [bb_cb_w(1, :); ...
             bb_cb_w(1, 1) bb_cb_w(2, 2); ...
             bb_cb_w(2, 1) bb_cb_w(1, 2); ...
             bb_cb_w(2, :)];

    % Apply xform
    box_s = [xform_w2s(:, 1:2) xform_w2s(:, 4)] * vertcat(box_w', ones(1, 4));
    box_s = box_s(1:3, :)';

    % Plot patch
    patch(a, box_s([1 2 4 3], 3), box_s([1 2 4 3], 1), box_s([1 2 4 3], 2), color, ...
          'FaceAlpha', alpha, 'EdgeAlpha', alpha, 'EdgeColor', 'k');
end
