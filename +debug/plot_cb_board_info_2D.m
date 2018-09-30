function plot_cb_board_info_2D(calib_config,a)
    % This will plot the calibration board points in world coordinates 
    % specified in calib_config.
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Get board points in world coordinates
    [p_cb_ws, four_points_w] = alg.p_cb_w(calib_config);
    
    % Format axes
    padding = calib_config.target_spacing/2;
    axis(a,'equal');
    set(a,'Ydir','reverse', ...
        'Xlim',[min(four_points_w(:,1))-padding, max(four_points_w(:,1)) + padding], ...        
        'Ylim',[min(four_points_w(:,2))-padding, max(four_points_w(:,2)) + padding]);  
    hold(a,'on');
        
    switch calib_config.calibration_target
        case 'checker'
            % Plot patches
            height_offset = (calib_config.height_fp-calib_config.num_targets_height*calib_config.target_spacing)/2;
            width_offset = (calib_config.width_fp-calib_config.num_targets_width*calib_config.target_spacing)/2;
            for i = 1:calib_config.num_targets_width
                for j = 1:calib_config.num_targets_height
                    % Get checker color; only odd # of checkers are allowed on
                    % each side, so this is ok
                    if ~util.is_even((i-1)*calib_config.num_targets_height+j)
                        patch_color = 'k';
                    else
                        patch_color = 'w';
                    end         

                    % Get checker coords
                    x_w = [(i-1)*calib_config.target_spacing (i-1)*calib_config.target_spacing ...
                           i*calib_config.target_spacing i*calib_config.target_spacing]'+width_offset;
                    y_w = [(j-1)*calib_config.target_spacing j*calib_config.target_spacing ...
                           j*calib_config.target_spacing (j-1)*calib_config.target_spacing]'+height_offset;

                    % Plot
                    patch(a,x_w,y_w,patch_color);
                end
            end        
        otherwise
            error(['Calibration target: "' calib_config.calibration_target '" is not supported.']);
    end
        
    % Plot axes
    axes_coords_w = [0 0;
                     calib_config.target_spacing 0;
                     0 0;
                     0 calib_config.target_spacing];
    quiver(axes_coords_w(1:2:end,1), ...
           axes_coords_w(1:2:end,2), ...
           axes_coords_w(2:2:end,1)-axes_coords_w(1:2:end,1), ...
           axes_coords_w(2:2:end,2)-axes_coords_w(1:2:end,2), ...
           'color','r','LineWidth',2,'AutoScale','off','parent',a);
       
    text_coords_w = [1.5*calib_config.target_spacing 0;
                     0 1.5*calib_config.target_spacing];
    text(text_coords_w(1,1),text_coords_w(1,2),'x', ...
         'FontSize',12,'HorizontalAlignment','center','color','r', ...
         'FontWeight','bold','parent',a);
    text(text_coords_w(2,1),text_coords_w(2,2),'y', ...
         'FontSize',12,'HorizontalAlignment','center','color','r', ...
         'FontWeight','bold','parent',a);
    
    % Plot board points    
    plot(p_cb_ws(:,1),p_cb_ws(:,2),'gs','MarkerSize',12, ...
         'MarkerFaceColor','w','parent',a);
    text(p_cb_ws(:,1),p_cb_ws(:,2),cellstr(num2str([1:size(p_cb_ws,1)]')), ...
         'FontSize',6,'HorizontalAlignment','center','color','k','parent',a); %#ok<NBRAK>
    
    % Plot four points 
    plot(four_points_w(:,1),four_points_w(:,2),'bo','MarkerSize',14, ...
         'MarkerFaceColor','w','LineWidth',1.5,'parent',a);
    text(four_points_w(:,1),four_points_w(:,2),cellstr(num2str([1:4]')), ...
         'FontSize',8,'HorizontalAlignment','center','parent',a); %#ok<NBRAK>    
     
    % Remove hold
    hold(a,'off');
end
