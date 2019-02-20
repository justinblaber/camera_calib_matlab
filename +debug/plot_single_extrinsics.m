function plot_single_extrinsics(Rs, ts, colors, alphas, opts, a)
    % This will plot single extrinsics

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

    % Plot calibration boards
    for i = 1:numel(Rs)
        % Get affine xform
        xform = [Rs{i} ts{i}; zeros(1, 3) 1];

        % Plot calibration board
        debug.plot_cb_3D(xform, colors(i, :), alphas(i), opts, a);
    end

    % Plot camera
    debug.plot_cam_3D(eye(4), 'k', 0.5, 1, 'b', 2, 10, opts, a);

    % Format plot
    set(a, 'Ydir', 'reverse');
    set(a, 'Zdir', 'reverse');
    daspect(a, [1 1 1]);
    grid(a, 'on');
    view(a, 3);
    axis(a, 'tight');

    % Remove hold
    hold(a, 'off');
end
