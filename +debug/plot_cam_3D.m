function plot_cam_3D(xform_12, face_color, face_alpha, edge_alpha, axes_color, axes_line_width, axes_font_size, opts, a)
    % This will plot a camera in 3D given an affine transform

    % Matlab's 3D plot is not very good; to get it in the orientation I want,
    % I've just switched the x, y, and z components with:
    %   x => y
    %   y => z
    %   z => x

    if ~exist('a', 'var')
        f = figure();
        a = axes(f);
    end

    % If camera size is not set (i.e. nan), then set the camera size to a
    % scalefactor of the calibration board target size.
    if isnan(opts.camera_size)
        camera_size = min(opts.cb_class.get_cb_height(), opts.cb_class.get_cb_width())/4;
    else
        camera_size = opts.camera_size;
    end

    % Set position of axes
    p_axes_1 = [0               0               0; ...
                2.0*camera_size 0               0; ...
                0               0               0; ...
                0               2.0*camera_size 0; ...
                0               0               0; ...
                0               0               2.0*camera_size];

    % Set position of axes text
    p_text_1 = [2.5*camera_size 0               0; ...
                0               2.5*camera_size 0; ...
                0               0               2.5*camera_size];

    % Create patches for camera; this is the "cone"
    cam_patch_1s{1} =  [ 0              0              0;
                         camera_size/2  camera_size/2  1.5*camera_size;
                        -camera_size/2  camera_size/2  1.5*camera_size];
    cam_patch_1s{2} =  [ 0              0              0;
                         camera_size/2  camera_size/2  1.5*camera_size;
                         camera_size/2 -camera_size/2  1.5*camera_size];
    cam_patch_1s{3} =  [ 0              0              0;
                         camera_size/2 -camera_size/2  1.5*camera_size;
                        -camera_size/2 -camera_size/2  1.5*camera_size];
    cam_patch_1s{4} =  [ 0              0              0;
                        -camera_size/2  camera_size/2  1.5*camera_size;
                        -camera_size/2 -camera_size/2  1.5*camera_size];
    % This is the camera body
    cam_patch_1s{5} =  [ camera_size/2  camera_size/2  camera_size/2;
                         camera_size/2  camera_size/2 -camera_size/2;
                        -camera_size/2  camera_size/2 -camera_size/2;
                        -camera_size/2  camera_size/2  camera_size/2];
    cam_patch_1s{6} =  [ camera_size/2 -camera_size/2  camera_size/2;
                         camera_size/2 -camera_size/2 -camera_size/2;
                        -camera_size/2 -camera_size/2 -camera_size/2;
                        -camera_size/2 -camera_size/2  camera_size/2];
    cam_patch_1s{7} =  [ camera_size/2  camera_size/2  camera_size/2;
                         camera_size/2  camera_size/2 -camera_size/2;
                         camera_size/2 -camera_size/2 -camera_size/2;
                         camera_size/2 -camera_size/2  camera_size/2];
    cam_patch_1s{8} =  [-camera_size/2  camera_size/2  camera_size/2;
                        -camera_size/2  camera_size/2 -camera_size/2;
                        -camera_size/2 -camera_size/2 -camera_size/2;
                        -camera_size/2 -camera_size/2  camera_size/2];
    cam_patch_1s{9} =  [ camera_size/2  camera_size/2  camera_size/2;
                         camera_size/2 -camera_size/2  camera_size/2;
                        -camera_size/2 -camera_size/2  camera_size/2;
                        -camera_size/2  camera_size/2  camera_size/2];
    cam_patch_1s{10} = [ camera_size/2  camera_size/2 -camera_size/2;
                         camera_size/2 -camera_size/2 -camera_size/2;
                        -camera_size/2 -camera_size/2 -camera_size/2;
                        -camera_size/2  camera_size/2 -camera_size/2];

    % Apply xform, then plot axes
    p_axes_2 = xform_12 * [p_axes_1 ones(size(p_axes_1, 1), 1)]';
    p_axes_2 = p_axes_2(1:3, :)';
    quiver3(p_axes_2(1:2:end, 3), p_axes_2(1:2:end, 1), p_axes_2(1:2:end, 2), ...
            p_axes_2(2:2:end, 3)-p_axes_2(1:2:end, 3), ...
            p_axes_2(2:2:end, 1)-p_axes_2(1:2:end, 1), ...
            p_axes_2(2:2:end, 2)-p_axes_2(1:2:end, 2), ...
            'AutoScale', 'off', 'LineWidth', axes_line_width, ...
            'MaxHeadSize', 0.4, 'color', axes_color, 'Parent', a);

    % Apply xform, then plot text for axes
    p_text_2 = xform_12 * [p_text_1 ones(size(p_text_1, 1), 1)]';
    p_text_2 = p_text_2(1:3, :)';
    text(a, p_text_2(1, 3), p_text_2(1, 1), p_text_2(1, 2), 'x', 'FontSize', axes_font_size, ...
         'Color', axes_color, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    text(a, p_text_2(2, 3), p_text_2(2, 1), p_text_2(2, 2), 'y', 'FontSize', axes_font_size, ...
         'Color', axes_color, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    text(a, p_text_2(3, 3), p_text_2(3, 1), p_text_2(3, 2), 'z', 'FontSize', axes_font_size, ...
         'Color', axes_color, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

    % Apply xform, then plot camera
    for i = 1:numel(cam_patch_1s)
        cam_patch_2 = xform_12 * [cam_patch_1s{i} ones(size(cam_patch_1s{i}, 1), 1)]';
        cam_patch_2 = cam_patch_2(1:3, :)';
        patch(a, cam_patch_2(:, 3), cam_patch_2(:, 1), cam_patch_2(:, 2), face_color, ...
              'FaceAlpha', face_alpha, 'EdgeColor', 'k', 'EdgeAlpha', edge_alpha);
    end
end
