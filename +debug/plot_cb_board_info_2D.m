function plot_cb_board_info_2D(cal_config,a)
    % This will plot the calibration board points in world coordinates 
    % specified in cal_config.
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Get board points in world coordinates
    [board_points, four_points] = alg.cb_points(cal_config);
    
    % Format axes
    padding = cal_config.square_size/2;
    axis(a,'equal');
    set(a,'Ydir','reverse', ...
        'Xlim',[min(four_points(:,1))-padding, max(four_points(:,1)) + padding], ...        
        'Ylim',[min(four_points(:,2))-padding, max(four_points(:,2)) + padding]);  
    hold(a,'on');
    
    % Plot patches
    height_offset = (cal_config.four_point_height-cal_config.num_squares_height*cal_config.square_size)/2;
    width_offset = (cal_config.four_point_width-cal_config.num_squares_width*cal_config.square_size)/2;
    for i = 1:cal_config.num_squares_width
        for j = 1:cal_config.num_squares_height
            % Get checker color; only odd # of checkers are allowed on
            % each side, so this is ok
            if ~util.is_even((i-1)*cal_config.num_squares_height+j)
                patch_color = 'k';
            else
                patch_color = 'w';
            end         
            
            % Get checker coords
            x_w = [(i-1)*cal_config.square_size (i-1)*cal_config.square_size ...
                   i*cal_config.square_size i*cal_config.square_size]'+width_offset;
            y_w = [(j-1)*cal_config.square_size j*cal_config.square_size ...
                   j*cal_config.square_size (j-1)*cal_config.square_size]'+height_offset;

            % Plot
            patch(a,x_w,y_w,patch_color);
        end
    end        
    
    % Plot axes
    axes_coords_w = [0 0;
                     cal_config.square_size 0;
                     0 0;
                     0 cal_config.square_size];
    quiver(axes_coords_w(1:2:end,1), ...
           axes_coords_w(1:2:end,2), ...
           axes_coords_w(2:2:end,1)-axes_coords_w(1:2:end,1), ...
           axes_coords_w(2:2:end,2)-axes_coords_w(1:2:end,2), ...
           'color','r','LineWidth',2,'AutoScale','off','parent',a);
       
    text_coords_w = [1.5*cal_config.square_size 0;
                     0 1.5*cal_config.square_size];
    text(text_coords_w(1,1),text_coords_w(1,2),'x', ...
         'FontSize',12,'HorizontalAlignment','center','color','r', ...
         'FontWeight','bold','parent',a);
    text(text_coords_w(2,1),text_coords_w(2,2),'y', ...
         'FontSize',12,'HorizontalAlignment','center','color','r', ...
         'FontWeight','bold','parent',a);
    
    % Plot board points    
    plot(board_points(:,1),board_points(:,2),'gs','MarkerSize',12, ...
         'MarkerFaceColor','w','parent',a);
    text(board_points(:,1),board_points(:,2),cellstr(num2str([1:size(board_points,1)]')), ...
         'FontSize',6,'HorizontalAlignment','center','color','k','parent',a); %#ok<NBRAK>
    
    % Plot four points 
    plot(four_points(:,1),four_points(:,2),'bo','MarkerSize',14, ...
         'MarkerFaceColor','w','LineWidth',1.5,'parent',a);
    text(four_points(:,1),four_points(:,2),cellstr(num2str([1:4]')), ...
         'FontSize',8,'HorizontalAlignment','center','parent',a); %#ok<NBRAK>    
     
    % Remove hold
    drawnow
    hold(a,'off');
end
