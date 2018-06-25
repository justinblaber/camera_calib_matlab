function plot_cb_board_3D(calib_config,xform,color,alpha,a)
    % This will plot a calibration board in 3D given an affine transform
        
    % Matlab's 3D plot is not very good; to get it in the orientation I want,
    % I've just switched the x, y, and z components with:
    %   x => y
    %   y => z
    %   z => x

    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
        
    % Plot board using patches
    patch_x_color = [];
    patch_y_color = [];
    patch_z_color = [];
    patch_x_white = [];
    patch_y_white = [];
    patch_z_white = [];
    height_offset = (calib_config.four_point_height-calib_config.num_targets_height*calib_config.target_spacing)/2;
    width_offset = (calib_config.four_point_width-calib_config.num_targets_width*calib_config.target_spacing)/2;
    for j = 1:calib_config.num_targets_width
        for k = 1:calib_config.num_targets_height            
            % Get checker coords
            x_w_cb = [(j-1)*calib_config.target_spacing (j-1)*calib_config.target_spacing ...
                      j*calib_config.target_spacing j*calib_config.target_spacing]+width_offset;
            y_w_cb = [(k-1)*calib_config.target_spacing k*calib_config.target_spacing ...
                      k*calib_config.target_spacing (k-1)*calib_config.target_spacing]+height_offset;
            
            % Apply xform
            p_s_cb = [xform(:,1:2) xform(:,4)] * vertcat(x_w_cb, y_w_cb, ones(1,4));
            p_s_cb = p_s_cb(1:3,:)';
            
            if ~util.is_even((j-1)*calib_config.num_targets_height+k)
                patch_x_color = horzcat(patch_x_color,p_s_cb(:,3)); %#ok<AGROW>
                patch_y_color = horzcat(patch_y_color,p_s_cb(:,1)); %#ok<AGROW>
                patch_z_color = horzcat(patch_z_color,p_s_cb(:,2)); %#ok<AGROW>
            else
                patch_x_white = horzcat(patch_x_white,p_s_cb(:,3)); %#ok<AGROW>
                patch_y_white = horzcat(patch_y_white,p_s_cb(:,1)); %#ok<AGROW>
                patch_z_white = horzcat(patch_z_white,p_s_cb(:,2)); %#ok<AGROW>
            end            

        end
    end 
    
    % Plot
    patch(a,patch_x_color,patch_y_color,patch_z_color,color, ...
          'FaceAlpha',alpha,'EdgeColor','k','EdgeAlpha',alpha);
    patch(a,patch_x_white,patch_y_white,patch_z_white,'white', ...
          'FaceAlpha',alpha,'EdgeColor','k','EdgeAlpha',alpha);
end
