function plot_camera_3D(xform,face_color,face_alpha,edge_alpha,axes_color,axes_line_width,axes_font_size,calib_config,a)
    % This will plot a camera in 3D given an affine transform
    
    % Matlab's 3D plot is not very good; to get it in the orientation I want,
    % I've just switched the x, y, and z components with:
    %   x => y
    %   y => z
    %   z => x
    
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    
    % If camera size is set to default, then set the camera size to the
    % calibration board target size.
    if calib_config.camera_size == eps
        camera_size = calib_config.target_spacing;
    else
        camera_size = calib_config.camera_size;
    end
        
    % Set position of axes
    p_axes = [0               0               0; ...
              2.0*camera_size 0               0; ...
              0               0               0; ...
              0               2.0*camera_size 0; ...
              0               0               0; ...
              0               0               2.0*camera_size];

    % Set position of axes text
    p_text = [2.5*camera_size 0               0; ...
              0               2.5*camera_size 0; ...
              0               0               2.5*camera_size];

    % Create patches for camera; this is the "cone"
    cam_patches{1} =  [ 0              0              0;
                        camera_size/2  camera_size/2  1.5*camera_size;
                       -camera_size/2  camera_size/2  1.5*camera_size];
    cam_patches{2} =  [ 0              0              0;
                        camera_size/2  camera_size/2  1.5*camera_size;
                        camera_size/2 -camera_size/2  1.5*camera_size];
    cam_patches{3} =  [ 0              0              0;
                        camera_size/2 -camera_size/2  1.5*camera_size;
                       -camera_size/2 -camera_size/2  1.5*camera_size];
    cam_patches{4} =  [ 0              0              0;
                       -camera_size/2  camera_size/2  1.5*camera_size;
                       -camera_size/2 -camera_size/2  1.5*camera_size];
    % This is the camera body
    cam_patches{5} =  [ camera_size/2  camera_size/2  camera_size/2;
                        camera_size/2  camera_size/2 -camera_size/2;
                       -camera_size/2  camera_size/2 -camera_size/2;
                       -camera_size/2  camera_size/2  camera_size/2];
    cam_patches{6} =  [ camera_size/2 -camera_size/2  camera_size/2;
                        camera_size/2 -camera_size/2 -camera_size/2;
                       -camera_size/2 -camera_size/2 -camera_size/2;
                       -camera_size/2 -camera_size/2  camera_size/2];
    cam_patches{7} =  [ camera_size/2  camera_size/2  camera_size/2;
                        camera_size/2  camera_size/2 -camera_size/2;
                        camera_size/2 -camera_size/2 -camera_size/2;
                        camera_size/2 -camera_size/2  camera_size/2];
    cam_patches{8} =  [-camera_size/2  camera_size/2  camera_size/2;
                       -camera_size/2  camera_size/2 -camera_size/2;
                       -camera_size/2 -camera_size/2 -camera_size/2;
                       -camera_size/2 -camera_size/2  camera_size/2];
    cam_patches{9} =  [ camera_size/2  camera_size/2  camera_size/2;
                        camera_size/2 -camera_size/2  camera_size/2;
                       -camera_size/2 -camera_size/2  camera_size/2;
                       -camera_size/2  camera_size/2  camera_size/2];
    cam_patches{10} = [ camera_size/2  camera_size/2 -camera_size/2;
                        camera_size/2 -camera_size/2 -camera_size/2;
                       -camera_size/2 -camera_size/2 -camera_size/2;
                       -camera_size/2  camera_size/2 -camera_size/2];

    % Apply xform, then plot axes    
    p_axes = xform * [p_axes ones(size(p_axes,1),1)]'; 
    p_axes = p_axes(1:3,:)';    
    quiver3(p_axes(1:2:end,3), p_axes(1:2:end,1), p_axes(1:2:end,2), ...
            p_axes(2:2:end,3)-p_axes(1:2:end,3), ...
            p_axes(2:2:end,1)-p_axes(1:2:end,1), ...
            p_axes(2:2:end,2)-p_axes(1:2:end,2), ...
            'AutoScale','off','LineWidth',axes_line_width, ...
            'MaxHeadSize',0.4,'color',axes_color,'Parent',a);

    % Apply xform, then plot text for axes
    p_text = xform * [p_text ones(size(p_text,1),1)]';
    p_text = p_text(1:3,:)';
    text(a,p_text(1,3),p_text(1,1),p_text(1,2),'x','FontSize',axes_font_size, ...
         'Color',axes_color,'HorizontalAlignment','center','VerticalAlignment','middle');
    text(a,p_text(2,3),p_text(2,1),p_text(2,2),'y','FontSize',axes_font_size, ...
         'Color',axes_color,'HorizontalAlignment','center','VerticalAlignment','middle');
    text(a,p_text(3,3),p_text(3,1),p_text(3,2),'z','FontSize',axes_font_size, ...
         'Color',axes_color,'HorizontalAlignment','center','VerticalAlignment','middle');

    % Apply xform, then plot camera
    for i = 1:numel(cam_patches)
        cam_patches{i} = xform * [cam_patches{i} ones(size(cam_patches{i},1),1)]';
        cam_patches{i} = cam_patches{i}(1:3,:)';
        patch(a,cam_patches{i}(:,3),cam_patches{i}(:,1),cam_patches{i}(:,2),face_color, ...
              'FaceAlpha',face_alpha,'EdgeColor','k','EdgeAlpha',edge_alpha);
    end
end
