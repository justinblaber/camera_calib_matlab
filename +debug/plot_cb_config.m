function plot_cb_config(cb_config,a)
    % This will plot the calibration board points in world coordinates 
    % specified in cb_config.
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    
    % Get board points in world coordinates
    [board_points, four_points] = alg.cb_points(cb_config);
    
    % Format axes
    axis(a,'equal');
    set(a,'Ydir','reverse', ...
        'Xlim',[min(four_points(:,1))-10, max(four_points(:,1)) + 10], ...        
        'Ylim',[min(four_points(:,2))-10, max(four_points(:,2)) + 10]);  
    hold(a,'on');
    
    % Plot patches
    height_offset = (cb_config.four_point_height-cb_config.num_squares_height*cb_config.square_size)/2;
    width_offset = (cb_config.four_point_width-cb_config.num_squares_width*cb_config.square_size)/2;
    for i = 1:cb_config.num_squares_width
        for j = 1:cb_config.num_squares_height
            if ~util.is_even((i-1)*cb_config.num_squares_height+j)
                x = [(i-1)*cb_config.square_size (i-1)*cb_config.square_size ...
                     i*cb_config.square_size i*cb_config.square_size];
                y = [(j-1)*cb_config.square_size j*cb_config.square_size ...
                     j*cb_config.square_size (j-1)*cb_config.square_size];
                patch(x+width_offset,y+height_offset,'black')
            end
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