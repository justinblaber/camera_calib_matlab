function [win_points_i, win_point_weights, win_point_corners_i] = refine_window_i(point_i,homography,width,height,cb_config)    
    % Computes refinement window points, weights of the window points, and 
    % the corners of the window points in image coordinates.
    %
    % Inputs:
    %   point_i - array; 1x2 point in image coordinates
    %   homography - array; 3x3 homography matrix. This is used to compute
    %       the window around each point.
    %   width - scalar; width of the calibration board image
    %   height - scalar; height of the calibration board image
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   win_points_i - array; points of refinement window in image
    %       coordinates
    %   win_point_weights - array; weights of refinement window
    %   win_point_corners_i - array; corners of refinement window. For now,
    %       just use these for plotting/debugging purposes. Currently, 
    %       these corners are not updated if refinement window is truncated
    %       due to being outside the image).
            
    % Get point in world coordinates
    point_w = alg.apply_homography(homography^-1,point_i);

    % Get window_factor
    wf = window_factor(point_w,...
                       homography, ...
                       cb_config);

    % Get half_window for sampling
    hw = half_window(point_w, ...
                     homography, ...
                     wf, ...
                     cb_config); 

    % Get window points in image coordinates
    win_points_i = window_points_i(point_w, ...
                                   homography, ...
                                   wf, ...
                                   hw, ...
                                   cb_config);                                       


    % Get weights for window_points
    win_point_weights = window_point_weights(hw);   
    
    % Get window point corners before thresholding based on image bounds.
    % Corners are stored as:
    %   p1 p3
    %   p2 p4
    l = 2*hw+1;
    win_point_corners_i = [win_points_i(1,1) win_points_i(1,2); ...
                           win_points_i(l,1) win_points_i(l,2); ...
                           win_points_i(l*l,1) win_points_i(l*l,2); ...
                           win_points_i(l*(l-1)+1,1) win_points_i(l*(l-1)+1,2)];
    
    % Make sure coords are within bounds
    idx_inbounds = win_points_i(:,1) >= 1 & win_points_i(:,1) <= width & ...
                   win_points_i(:,2) >= 1 & win_points_i(:,2) <= height;
               
    % Only keep inbound idx
    win_points_i = win_points_i(idx_inbounds,:);        
    win_point_weights = win_point_weights(idx_inbounds,:);
    
    % TODO: Possibly win_point_corners_i in the future if the window is out
    % of bounds
end

function l_i = window_lengths_i(point_w,homography,wf,cb_config)
    % Computes a window, in image coordinates, using input point in world
    % coordinates, homography, window_factor and cb_config, then calculates
    % the lengths of each side of the window.
    %
    %   Points and lengths are:
    %       p1 - l2 - p3
    %       |         |
    %       l1   p_w  l4
    %       |         |
    %       p2 - l3 - p4
    
    % Get points in world coordinates
    p1_w = [point_w(1)-(cb_config.square_size/2)*wf, ...
            point_w(2)-(cb_config.square_size/2)*wf];
    p2_w = [point_w(1)-(cb_config.square_size/2)*wf, ...
            point_w(2)+(cb_config.square_size/2)*wf];
    p3_w = [point_w(1)+(cb_config.square_size/2)*wf, ...
            point_w(2)-(cb_config.square_size/2)*wf];
    p4_w = [point_w(1)+(cb_config.square_size/2)*wf, ...
            point_w(2)+(cb_config.square_size/2)*wf];
        
    % Apply homography
    p_win_i = alg.apply_homography(homography,vertcat(p1_w,p2_w,p3_w,p4_w));
    
    % Calculate distances
    l_i(1) = norm(p_win_i(2,:)-p_win_i(1,:));
    l_i(2) = norm(p_win_i(3,:)-p_win_i(1,:));
    l_i(3) = norm(p_win_i(4,:)-p_win_i(2,:));
    l_i(4) = norm(p_win_i(4,:)-p_win_i(3,:));
end

function wf = window_factor(point_w,homography,cb_config)
    % Computes the window factor, which is a proportion of the checkerboard
    % square used to compute the refinement window. This will either:
    %   Return the default window factor if it meets the minimum length
    %       requirement and is less than 4/3
    %   Return a newly computed window factor which ensures the minimum
    %       length of the refinement window is 
    %       refine_corner_window_min_size if the default refinement window 
    %       doesnt meet this criteria
    %   Return 4/3, which is the upper bound I set to ensure the refinement
    %       window does not overlap with neighboring corners
    
    % Initialize window factor
    wf = cb_config.refine_corner_default_window_factor;
        
    % Get window lengths in image coordinates
    l_i = window_lengths_i(point_w, ...
                           homography, ...
                           wf, ...
                           cb_config);
        
    % Recompute window_factor if any of the distances are below the minimum
    % window size
    if any(l_i < cb_config.refine_corner_window_min_size)
        disp('WARNING: min window constraint met; recomputing window factor for this corner.');
        
        [~, min_idx] = min(l_i);
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
        
        % Equations boil down to 4th order polynomial - this may not be the
        % most optimal way to do this, but it works.
        a = p1_dir(1)*homography(1,1)+p1_dir(2)*homography(1,2);
        b = p2_dir(1)*homography(1,1)+p2_dir(2)*homography(1,2);
        c = p1_dir(1)*homography(2,1)+p1_dir(2)*homography(2,2);
        d = p2_dir(1)*homography(2,1)+p2_dir(2)*homography(2,2);
        e = p1_dir(1)*homography(3,1)+p1_dir(2)*homography(3,2);
        f = p2_dir(1)*homography(3,1)+p2_dir(2)*homography(3,2);
        j = homography(1,1)*point_w(1)+homography(1,2)*point_w(2)+homography(1,3);
        k = homography(2,1)*point_w(1)+homography(2,2)*point_w(2)+homography(2,3);
        l = homography(3,1)*point_w(1)+homography(3,2)*point_w(2)+homography(3,3);
        r = roots([cb_config.refine_corner_window_min_size^2*f^2*e^2-(a*f-e*b)^2-(c*f-e*d)^2 ...
                   2*cb_config.refine_corner_window_min_size^2*f*e*(l*f+l*e)-2*(a*f-e*b)*(f*j+l*a-e*j-l*b)-2*(c*f-e*d)*(f*k+l*c-e*k-l*d) ...
                   2*cb_config.refine_corner_window_min_size^2*l^2*f*e+cb_config.refine_corner_window_min_size^2*(l*f+l*e)^2-(f*j+l*a-e*j-l*b)^2-(f*k+l*c-e*k-l*d)^2 ...
                   2*cb_config.refine_corner_window_min_size^2*l^2*(l*f+l*e) ...
                   cb_config.refine_corner_window_min_size^2*l^4]);

        % Get smallest, real, and positive root to get window_factor.
        wf = min(r(arrayfun(@(x)isreal(x(1)),r) & r > 0));
        wf = 2*wf/cb_config.square_size;
    end
    
    % Threshold window_factor to 4/3 to prevent overlap
    if wf >= 4/3
        disp('WARNING: max window_factor is being set.');
        wf = 4/3;
    end
end

function hw = half_window(point_w,homography,wf,cb_config)
    % Computes half window used for refinement window
        
    hw = floor(max(window_lengths_i(point_w,homography,wf,cb_config))/4)*2+1;
end

function win_points_i = window_points_i(point_w, homography, wf, hw, cb_config)
    % Computes window points in image coordinates
    
    % Get grid of points in world coordinates
    [win_points_y, win_points_x] = ndgrid(linspace(point_w(2)-(cb_config.square_size/2)*wf, ...
                                                   point_w(2)+(cb_config.square_size/2)*wf, ...
                                                   2*hw+1), ...
                                          linspace(point_w(1)-(cb_config.square_size/2)*wf, ...
                                                   point_w(1)+(cb_config.square_size/2)*wf, ...
                                                   2*hw+1));        
    win_points_w = [win_points_x(:) win_points_y(:)];        
    
    % Apply homography to window points to bring them into image
    % coordinates
    win_points_i = alg.apply_homography(homography,win_points_w);
end

function weights = window_point_weights(hw)
    % Computes weights of window points.
       
    % TODO: possibly add sigma parameter to cb config file
    
    % Get gaussian kernel
    weights = fspecial('Gaussian',[2*hw+1 2*hw+1],hw);
              
    % Scale so max intensity is 1
    weights = reshape(weights./max(weights(:)),[],1);
end