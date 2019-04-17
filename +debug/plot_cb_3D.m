function plot_cb_3D(obj_cb_geom, xform_w2s, color, alpha, a)
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

    if ~isa(obj_cb_geom, 'class.cb_geom.size_intf')
        error('calibration board geometry must inherit from size interface to use this function');
    end

    % Form box
    % Note:
    %    p1 - p3
    %     |    |
    %    p2 - p4
    h = obj_cb_geom.get_cb_height();
    w = obj_cb_geom.get_cb_width();
    box_w = [0 0; ...
             0 h; ...
             w 0; ...
             w h];

    % Apply xform
    box_s = [xform_w2s(:, 1:2) xform_w2s(:, 4)] * vertcat(box_w', ones(1, 4));
    box_s = box_s(1:3, :)';

    % Plot patch
    patch(a, box_s([1 2 4 3], 3), box_s([1 2 4 3], 1), box_s([1 2 4 3], 2), color, ...
          'FaceAlpha', alpha, 'EdgeAlpha', alpha, 'EdgeColor', 'k');
end
