function points = refine_points(points,cb_img,homography,cb_config)
    % This will return refined point coordinates given an initial guess for
    % the points, calibration board image, initial guess of the homography,
    % and calibration board config. This uses the technique of getting the 
    % dot product, along a window, of the displacement vector and gradient.
    %
    % Inputs:
    %   points - array; nx2 array of points
    %   cb_img - class.img; calibration board image
    %   homography - array; 3x3 homography matrix of initial guess. This is
    %       used to compute window around each point.
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   points - array; nx2 array of refined points.
        
    % Get calibration board image
    cb_gs = cb_img.get_gs();
        
    % Get gradient images
    cb_gs_dx = alg.array_grad(cb_gs,'x');
    cb_gs_dy = alg.array_grad(cb_gs,'y');
                               
    % Perform iterations until convergence
    it_cutoff = 10;
    norm_cutoff = 0.05;
    for i = 1:size(points,1)           
        for it = 1:it_cutoff
            % Get point            
            point_prev = points(i,:);            
            
            % Get window around point; apply inverse homography to it to
            % first bring it into world coordinates
            win_points_w = window_points(alg.apply_inv_homography(homography,point_prev),cb_config); 
        
            % Apply homography to window points to bring them into image
            % coordinates
            win_points_i = alg.apply_homography(homography,win_points_w);
            
            % Make sure coords are withinbounds
            idx_inbounds = win_points_i(:,1) >= 1 & win_points_i(:,1) <= size(cb_gs,2) & ...
                           win_points_i(:,2) >= 1 & win_points_i(:,2) <= size(cb_gs,1);
            win_points_i = win_points_i(idx_inbounds,:);
                
            % Refine point
            points(i,:) = refine_points_it(point_prev,cb_gs_dx,cb_gs_dy,win_points_i);  
            
            % Exit if change in distance is small
            if norm(point_prev-points(i,:)) < norm_cutoff
                break
            end
        end  
        disp(['Iterations: ' num2str(it)]);
        disp(['Norm: ' num2str(norm(point_prev-points(i,:)))]);
    end    
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

function points = refine_points_it(points,cb_gs_dx,cb_gs_dy,win_points)
    cb_gs_dx_window = alg.array_interp(cb_gs_dx,win_points,'cubic');
    cb_gs_dy_window = alg.array_interp(cb_gs_dy,win_points,'cubic');

    A = [cb_gs_dx_window cb_gs_dy_window];
    b = (cb_gs_dx_window.*win_points(:,1) + cb_gs_dy_window.*win_points(:,2));

    points(1,:) = (pinv(A)*b)';  
end