function plot_cb_refine_points(points,cb_img,homography,cb_config,show_weights,a)
    % This will plot the points and the refinement window around each 
    % point.
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
             
    % Plot points over calibration board image    
    cb_img.imshow(a);
    hold(a,'on');
    plot(points(:,1),points(:,2),'r+','parent',a);
    
    % Plot boxes around points   
    for i = 1:size(points,1)          
        % Get window points in image coordinates
        [win_points_p,win_point_weights,win_point_corners] = alg.refine_window_p(points(i,:), ...
                                                                                 homography, ...
                                                                                 cb_img.get_width(), ...
                                                                                 cb_img.get_height(), ...
                                                                                 cb_config);
        
        % Plot points
        if exist('show_weights','var') && show_weights
            % Plot window points with weights - this is slow
            for j = 1:size(win_points_p,1)
                plot(win_points_p(j,1),win_points_p(j,2),'bo','Markersize',5*win_point_weights(j),'parent',a);
            end   
            drawnow
        else
            % Only plot corners
            plot(vertcat(win_point_corners(:,1),win_point_corners(1,1)), ...
                 vertcat(win_point_corners(:,2),win_point_corners(1,2)), ...
                 'b','parent',a,'LineWidth',0.25);
        end     
    end        
    drawnow
end
