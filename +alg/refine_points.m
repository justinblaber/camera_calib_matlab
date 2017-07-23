function points = refine_points(points,cb_img,homography,cb_config)
    % This will return refined point coordinates given an initial guess for
    % the points, calibration board image, initial guess of the homography,
    % and calibration board config. This uses the technique of getting the 
    % dot product, in a window, of the displacement vector and gradient,
    % which should be zero for all points within the window.
    %
    % Inputs:
    %   points - array; Nx2 array of points
    %   cb_img - class.img; calibration board image
    %   homography - array; 3x3 homography matrix of initial guess. This is
    %       used to compute the window around each point.
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
    disp(['Refining points for: ' cb_img.get_path() '...']);
    for i = 1:size(points,1)           
        for it = 1:cb_config.refine_it_cutoff
            % Get point            
            point_prev = points(i,:);   
            
            % Get point in world coordinates
            point_prev_w = alg.apply_homography(homography^-1,point_prev);
            
            % Get window_factor
            wf = window_factor(point_prev_w,...
                               homography, ...
                               cb_config);
            
            % Get half_window
            hw = half_window(homography,cb_config);

            % Get gaussian kernel - this is used to weight points closer to the
            % corner higher.
            gk = gauss_kernel(hw);    
            
            % Get window around point; apply inverse homography to it in 
            % order to first bring it into world coordinates
            win_points_w = window_points(alg.apply_homography(homography^-1,point_prev), ...
                                         hw, ...
                                         cb_config); 
        
                                     
                                     
            % Apply homography to window points to bring them into image
            % coordinates
            win_points_i = alg.apply_homography(homography,win_points_w);
            
            % Make sure coords are withinbounds
            idx_inbounds = win_points_i(:,1) >= 1 & win_points_i(:,1) <= size(cb_gs,2) & ...
                           win_points_i(:,2) >= 1 & win_points_i(:,2) <= size(cb_gs,1);
            win_points_i = win_points_i(idx_inbounds,:);
                
            % Refine point
            points(i,:) = refine_point_it(cb_gs_dx, ...
                                          cb_gs_dy, ...
                                          win_points_i, ...                                          
                                          gk);  
            
            % Exit if change in distance is small
            if norm(point_prev-points(i,:)) < cb_config.refine_norm_cutoff
                break
            end
        end  
        if it == cb_config.refine_it_cutoff
            disp('WARNING: iterations hit cutoff before converging!!!');
        else
            disp(['Iterations: ' num2str(it)]);
        end
        disp(['Pixel difference norm: ' num2str(norm(point_prev-points(i,:)))]);
    end    
    disp('--------------------------------------------');
end

function p_win_i = window_corners_i(point_w,homography,cb_config)
    % Compute window using default window_factor; window is:
    % p1 p3
    % p2 p4
    p1_w = [point_w(1)-(cb_config.square_size/2)*cb_config.refine_window_factor, ...
          point_w(2)-(cb_config.square_size/2)*cb_config.refine_window_factor];
    p2_w = [point_w(1)-(cb_config.square_size/2)*cb_config.refine_window_factor, ...
          point_w(2)+(cb_config.square_size/2)*cb_config.refine_window_factor];
    p3_w = [point_w(1)+(cb_config.square_size/2)*cb_config.refine_window_factor, ...
          point_w(2)-(cb_config.square_size/2)*cb_config.refine_window_factor];
    p4_w = [point_w(1)+(cb_config.square_size/2)*cb_config.refine_window_factor, ...
          point_w(2)+(cb_config.square_size/2)*cb_config.refine_window_factor];
        
    % Apply homography
    p_win_i = alg.apply_homography(homography,vertcat(p1_w,p2_w,p3_w,p4_w));
end

function wf = window_factor(point_w,homography,cb_config)
    % Initialize window factor
    wf = cb_config.refine_window_factor;
        
    % Get points of window corners in image coordinates
    p_win_i = window_corners_i(point_w,homography,cb_config);
    
    % Calculate min distance
    d(1) = norm(p_win_i(2,:)-p_win_i(1,:));
    d(2) = norm(p_win_i(3,:)-p_win_i(1,:));
    d(3) = norm(p_win_i(4,:)-p_win_i(2,:));
    d(4) = norm(p_win_i(4,:)-p_win_i(3,:));
    
    % Recompute window_factor if any of the distances are below the minimum
    % window size
    if any(d < cb_config.refine_window_min_size)
        [~, min_idx] = min(d);
        switch min_idx
            case 1
                p1_dir = [-1 -1];
                p2_dir = [-1  1];   
            case 2
                p1_dir = [-1 -1];
                p2_dir = [ 1 -1];   
            case 3
                p1_dir = [-1  1];
                p2_dir = [ 1  1];  
            case 4 
                p1_dir = [ 1 -1];
                p2_dir = [ 1  1];  
        end
        
        % Equations boil down to 4th order polynomial
        a = p1_dir(1)*homography(1,1)+p1_dir(2)*homography(1,2);
        b = p2_dir(1)*homography(1,1)+p2_dir(2)*homography(1,2);
        c = p1_dir(1)*homography(2,1)+p1_dir(2)*homography(2,2);
        d = p2_dir(1)*homography(2,1)+p2_dir(2)*homography(2,2);
        e = p1_dir(1)*homography(3,1)+p1_dir(2)*homography(3,2);
        f = p2_dir(1)*homography(3,1)+p2_dir(2)*homography(3,2);
        j = homography(1,1)*point_w(1)+homography(1,2)*point_w(2)+homography(1,3);
        k = homography(2,1)*point_w(1)+homography(2,2)*point_w(2)+homography(2,3);
        l = homography(3,1)*point_w(1)+homography(3,2)*point_w(2)+homography(3,3);
        r = roots([cb_config.refine_window_min_size^2*f^2*e^2-(a*f-e*b)^2-(c*f-e*d)^2 ...
                   2*cb_config.refine_window_min_size^2*f*e*(l*f+l*e)-2*(a*f-e*b)*(f*j+l*a-e*j-l*b)-2*(c*f-e*d)*(f*k+l*c-e*k-l*d) ...
                   2*cb_config.refine_window_min_size^2*l^2*f*e+cb_config.refine_window_min_size^2*(l*f+l*e)^2-(f*j+l*a-e*j-l*b)^2-(f*k+l*c-e*k-l*d)^2 ...
                   2*cb_config.refine_window_min_size^2*l^2*(l*f+l*e) ...
                   cb_config.refine_window_min_size^2*l^4]);

        % Get smallest, real, and positive root to get window_factor.
        wf = min(r(arrayfun(@(x)isreal(x(1)),r) & r > 0));
        wf = 2*wf/cb_config.square_size;
    end
    
    % Threshold window_factor to 4/3 to prevent overlap
    if wf > 4/3
        wf = 4/3;
    end
end

function hw = half_window(homography,cb_config)
    % Get half window such that there is at least one sample per pixel
    % Get board points in world coordinates    
    board_points_w = alg.cb_points(cb_config);
    h = cb_config.num_rects_height;
    w = cb_config.num_rects_width;
        
    % Get outside points of four corner rectangles
    crp_w = vertcat(board_points_w(1,:), ...             % top-left
                    board_points_w(2,:), ...             % top-left-bottom
                    board_points_w(h,:), ...             % bottom-left-top
                    board_points_w(h+1,:), ...           % bottom-left
                    board_points_w(h+2,:), ...           % top-left-right                  
                    board_points_w((h+1)*2,:), ...       % bottom-left-right
                    board_points_w((w-1)*(h+1)+1,:), ... % top-right-left   
                    board_points_w(w*(h+1),:), ...       % bottom-right-left
                    board_points_w(w*(h+1)+1,:), ...     % top-right
                    board_points_w(w*(h+1)+2,:), ...     % top-right-bottom
                    board_points_w((w+1)*(h+1)-1,:), ... % bottom-right-top
                    board_points_w((w+1)*(h+1),:));      % bottom-right
                  
    % Apply homography
    crp_i = alg.apply_homography(homography,crp_w);
   
    % Get distances of outside sides of corner rectangles
    crd_i = vertcat(norm(crp_i(2,:) - crp_i(1,:)), ...  % tl - tlb
                    norm(crp_i(4,:) - crp_i(3,:)), ...  % blt - bl
                    norm(crp_i(5,:) - crp_i(1,:)), ...  % tlr - tl
                    norm(crp_i(6,:) - crp_i(4,:)), ...  % blr - bl
                    norm(crp_i(9,:) - crp_i(7,:)), ...  % tr - trl
                    norm(crp_i(12,:) - crp_i(8,:)), ... % br - brl
                    norm(crp_i(10,:) - crp_i(9,:)), ... % trb - tr
                    norm(crp_i(12,:) - crp_i(11,:)));   % br - brt
            
    % Set half-window based on largest length
    hw = ceil(max(crd_i*cb_config.refine_window_factor/2));
end

function gk = gauss_kernel(half_window)
    % Get gaussian kernel
    gk = fspecial('Gaussian', ...
                  [2*half_window+1 2*half_window+1], ...
                  half_window);
              
    % Scale so max intensity is 1
    gk = reshape(gk./max(gk(:)),[],1);
end

function win_points = window_points(point_w,half_window,cb_config)
    % Get grid of points in world coordinates
    [win_points_y, win_points_x] = ndgrid(linspace(point_w(2)-(cb_config.square_size/2)*cb_config.refine_window_factor, ...
                                                   point_w(2)+(cb_config.square_size/2)*cb_config.refine_window_factor, ...
                                                   2*half_window+1), ...
                                          linspace(point_w(1)-(cb_config.square_size/2)*cb_config.refine_window_factor, ...
                                                   point_w(1)+(cb_config.square_size/2)*cb_config.refine_window_factor, ...
                                                   2*half_window+1));        
    win_points = [win_points_x(:) win_points_y(:)];    
end

function point = refine_point_it(cb_gs_dx,cb_gs_dy,points_win,weights)
    % Interpolate gradients
    cb_gs_dx_window = alg.array_interp(cb_gs_dx,points_win,'cubic');
    cb_gs_dy_window = alg.array_interp(cb_gs_dy,points_win,'cubic');
    
    % Apply weights
    cb_gs_dx_window = cb_gs_dx_window.*weights;
    cb_gs_dy_window = cb_gs_dy_window.*weights;

    % Form linear system
    A = [cb_gs_dx_window cb_gs_dy_window];
    b = (cb_gs_dx_window.*points_win(:,1) + cb_gs_dy_window.*points_win(:,2));

    % Solve for updated point
    point = (pinv(A)*b)';  
end