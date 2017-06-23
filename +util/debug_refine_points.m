function debug_refine_points(points,cb_img,homography,cb_config,a)
    % This will plot the points and the window around the points
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
             
    % Plot points over calibration board image    
    cb_img.imshow(a);
    hold(a,'on');
    plot(points(:,1),points(:,2),'go','parent',a);
    
    % Plot boxes around points   
    cb_gs = cb_img.get_gs();
    for i = 1:size(points,1)    
        % Get window around point; apply inverse homography to it to
        % first bring it into world coordinates
        win_points_w = window_points(alg.apply_inv_homography(homography,points(i,:)),cb_config); 

        % Apply homography to window points to bring them into image
        % coordinates
        win_points_i = alg.apply_homography(homography,win_points_w);

        % Make sure coords are withinbounds
        idx_inbounds = win_points_i(:,1) >= 1 & win_points_i(:,1) <= size(cb_gs,2) & ...
                       win_points_i(:,2) >= 1 & win_points_i(:,2) <= size(cb_gs,1);
        win_points_i = win_points_i(idx_inbounds,:);
        
        % Plot points
        plot(win_points_i(:,1),win_points_i(:,2),'ro','Markersize',1,'parent',a);
    end
        
    drawnow;
end

function win_points = window_points(point_w,cb_config)
    num_points = 30;
    h = cb_config.rect_height;
    w = cb_config.rect_width;

    % order is: 1 4
    %           2 3
    win_points_x = [repmat(point_w(1)-w/3,1,num_points) linspace(point_w(1)-w/3,point_w(1)+w/3,num_points) repmat(point_w(1)+w/3,1,num_points) linspace(point_w(1)+w/3,point_w(1)-w/3,num_points)]';
    win_points_y = [linspace(point_w(2)-h/3,point_w(2)+h/3,num_points) repmat(point_w(2)+h/3,1,num_points) linspace(point_w(2)+h/3,point_w(2)-h/3,num_points) repmat(point_w(2)-h/3,1,num_points)]';
        
    % Corners have been lazily duplicated; use unique to get rid of
    % duplicates.
    win_points = unique([win_points_x win_points_y],'rows');    
end