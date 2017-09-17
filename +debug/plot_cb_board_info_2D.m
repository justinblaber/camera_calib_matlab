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
    axis(a,'equal');
    set(a,'Ydir','reverse', ...
        'Xlim',[min(four_points(:,1))-10, max(four_points(:,1)) + 10], ...        
        'Ylim',[min(four_points(:,2))-10, max(four_points(:,2)) + 10]);  
    hold(a,'on');
    
    % Plot patches
    height_offset = (cal_config.four_point_height-cal_config.num_squares_height*cal_config.square_size)/2;
    width_offset = (cal_config.four_point_width-cal_config.num_squares_width*cal_config.square_size)/2;
    for i = 1:cal_config.num_squares_width
        for j = 1:cal_config.num_squares_height
            % Get checker color
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
    hold(a,'off');
    
    drawnow;
end
