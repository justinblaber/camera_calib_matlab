function plot_multi_extrinsics(Rs, ts, R_1s, t_1s, colors, alphas, opts, a)
    % This will plot multi extrinsics

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
    
    % Get number of cameras and boards
    num_cams = numel(R_1s);
    num_boards = numel(Rs);

    % Plot calibration boards; xform is applied to get the calibration
    % boards in the coordinates of the first camera.
    for i = 1:num_boards
        % Get affine xform
        xform = [Rs{i} ts{i}; zeros(1, 3) 1];

        % Plot calibration board
        debug.plot_cb_3D(xform, colors(i, :), alphas(i), opts, a);
    end

    % Plot cameras
    for i = 1:num_cams
        xform = inv([R_1s{i} t_1s{i}; zeros(1, 3) 1]);
        debug.plot_cam_3D(xform, 'k', 0.5, 1, 'b', 2, 10, opts, a);
    end

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
