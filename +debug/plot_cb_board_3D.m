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
    height_offset = (calib_config.four_point_height-calib_config.num_squares_height*calib_config.square_size)/2;
    width_offset = (calib_config.four_point_width-calib_config.num_squares_width*calib_config.square_size)/2;
    for j = 1:calib_config.num_squares_width
        for k = 1:calib_config.num_squares_height
            % Get checker color; only odd # of checkers are allowed on
            % each side, so this is ok
            if ~util.is_even((j-1)*calib_config.num_squares_height+k)
                patch_color = color;
            else
                patch_color = 'white';
            end            
            
            % Get checker coords
            x_w_cb = [(j-1)*calib_config.square_size (j-1)*calib_config.square_size ...
                      j*calib_config.square_size j*calib_config.square_size]+width_offset;
            y_w_cb = [(k-1)*calib_config.square_size k*calib_config.square_size ...
                      k*calib_config.square_size (k-1)*calib_config.square_size]+height_offset;
            
            % Apply xform
            p_s_cb = [xform(:,1:2) xform(:,4)] * vertcat(x_w_cb, y_w_cb, ones(1,4));
            p_s_cb = p_s_cb(1:3,:)';
            
            % Plot
            patch(a,p_s_cb(:,3),p_s_cb(:,1),p_s_cb(:,2),patch_color, ...
                  'FaceAlpha',alpha,'EdgeColor','k','EdgeAlpha',alpha);
        end
    end 
end
