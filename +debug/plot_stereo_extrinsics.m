function plot_stereo_extrinsics(Rs, ts, R_s, t_s, colors, alphas, opts, a)
    % This will plot stereo extrinsics

    % Matlab's 3D plot is not very good; to get it in the orientation I want,
    % I've just switched the x, y, and z components with:
    %   x => y
    %   y => z
    %   z => x

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    % Hold
    hold(a, 'on');

    % Plot calibration boards; xform is applied to get the calibration
    % boards in the coordinates of the left camera.
    for i = 1:numel(Rs.L)
        % Get affine xform
        xform = [Rs.L{i} ts.L{i}; zeros(1, 3) 1];

        % Plot calibration board
        debug.plot_cb_3D(xform, colors(i, :), alphas(i), opts, a);

        % Plot text
        debug.plot_text_3D(num2str(i), ...
                           -2*opts.target_spacing, ...
                           -2*opts.target_spacing, ...
                           0, ...
                           xform, ...
                           colors(i, :), ...
                           10, ...
                           'bold', ...
                           a);
    end

    % Plot left camera
    debug.plot_cam_3D(eye(4), 'k', 0.5, 1, 'b', 2, 10, opts, a);

    % Plot right camera
    xform = inv([R_s t_s; zeros(1, 3) 1]);
    debug.plot_cam_3D(xform,  'k', 0.5, 1, 'r', 2, 10, opts, a);

    % Format plot
    set(a, 'Ydir', 'reverse');
    set(a, 'Zdir', 'reverse');
    daspect(a, [1 1 1]);
    grid(a, 'on');
    view(a, 3)
    axis(a, 'tight');

    % Remove hold
    hold(a, 'off');
end
