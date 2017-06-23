function debug_cb_config(cb_config,a)
    % This will plot the calibration board points in world coordinates 
    % specified in cb_config.
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    
    % Get board points in world coordinates
    [board_points, four_points] = alg.cb_points(cb_config);
    
    % Plot board points    
    plot(board_points(:,1),board_points(:,2),'bo','parent',a,'MarkerSize',10);
    text(board_points(:,1),board_points(:,2),cellstr(num2str([1:size(board_points,1)]')),'parent',a,'FontSize',6,'HorizontalAlignment','center'); %#ok<NBRAK>
    axis(a,'equal');
    set(a,'Ydir','reverse', ...
        'Xlim',[min(four_points(:,1))-10, max(four_points(:,1)) + 10], ...        
        'Ylim',[min(four_points(:,2))-10, max(four_points(:,2)) + 10]);  
    hold(a,'on');
    
    % Plot four points 
    plot(four_points(:,1),four_points(:,2),'rs','parent',a,'MarkerSize',14,'LineWidth',1.5);
    text(four_points(:,1),four_points(:,2),cellstr(num2str([1:4]')),'parent',a,'FontSize',8,'HorizontalAlignment','center'); %#ok<NBRAK>    
    hold(a,'off');
    
    drawnow;
end