function plot_stereo_extrinsic(rotations,translations,R_s,t_s,colors,alphas,cal_config,a)
    % This will plot extrinsics for a stero camera rig
    
    % Matlab's 3D plot is not very good; to get it in the orientation I want,
    % I've just switched the x, y, and z components with:
    %   x => y
    %   y => z
    %   z => x
    
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
        
    % Hold
    hold(a,'on');

    % Plot checker boards
    % Note that xform is applied to get the checkerboard in the coordinates of 
    % the left camera
    height_offset = (cal_config.four_point_height-cal_config.num_squares_height*cal_config.square_size)/2;
    width_offset = (cal_config.four_point_width-cal_config.num_squares_width*cal_config.square_size)/2;
    for i = 1:length(rotations.L)    
        % Get affine xform
        xform = [rotations.L{i} translations.L{i}; zeros(1,3) 1];

        % Plot calibration board
        debug.plot_cb_board_3D(cal_config, ...
                               xform, ...
                               colors(i,:), ...
                               alphas(i), ...
                               a);

        % Plot text   
        debug.plot_text_3D(num2str(i), ...
                           width_offset-cal_config.square_size/2, ...
                           height_offset-cal_config.square_size/2, ...
                           0, ...
                           xform, ...
                           colors(i,:), ...
                           10, ...
                           'bold', ...
                           a);
    end

    % Plot left camera
    debug.plot_camera_3D(eye(4), ...
                         'k', ...
                         0.5, ...
                         1, ...
                         'b', ...
                         2, ...
                         10, ...
                         cal_config, ...
                         a);

    % Plot right camera
    xform_s_inv = inv([R_s t_s; zeros(1,3) 1]);
    debug.plot_camera_3D(xform_s_inv, ...
                         'k', ...
                         0.5, ...
                         1, ...
                         'r', ...
                         2, ...
                         10, ...
                         cal_config, ...
                         a);

    % Format plot
    set(a,'Ydir','reverse');
    set(a,'Zdir','reverse');
    daspect(a,[1 1 1]);
    grid(a,'on');
    view(a,3)
    axis(a,'tight');
    
    % Remove hold
    drawnow
    hold(a,'off');
end