function [win_points_p, win_point_weights, win_point_corners_p] = refine_window_p(point_p,homography,homography_inv,width,height,calib_config)    
    % Computes refinement window points, weights of the window points, and 
    % the corners of the window points in pixel coordinates.
    %
    % Inputs:
    %   point_p - array; 1x2 point in pixel coordinates
    %   homography - array; 3x3 homography matrix. This is used to compute
    %       the window around each point.
    %   homography_inv - array; 3x3 inverse homography matrix. This is
    %       passed for speed, instead of recomputing inverse.
    %   width - scalar; width of the calibration board image
    %   height - scalar; height of the calibration board image
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   win_points_p - array; points of refinement window in pixel
    %       coordinates
    %   win_point_weights - array; weights of refinement window
    %   win_point_corners_p - array; corners of refinement window. For now,
    %       just use these for plotting/debugging purposes. Currently, 
    %       these corners are not updated if refinement window is truncated
    %       due to being outside the image.
            
    % Get point in world coordinates
    point_w = alg.apply_homography(homography_inv,point_p);

    % Get window_factor
    wf = window_factor(point_w,...
                       homography, ...
                       calib_config);

    % Get half_window for sampling
    hw = half_window(point_w, ...
                     homography, ...
                     wf, ...
                     calib_config); 

    % Get window points in pixel coordinates
    win_points_p = window_points_p(point_w, ...
                                   homography, ...
                                   wf, ...
                                   hw, ...
                                   calib_config);             

    % Get weights for window_points
    win_point_weights = window_point_weights(hw);   
    
    % Get window point corners before thresholding.
    % Corners are stored as:
    %   p1 p3
    %   p2 p4
    l = 2*hw+1;
    win_point_corners_p = [win_points_p(1,1) win_points_p(1,2); ...
                           win_points_p(l,1) win_points_p(l,2); ...
                           win_points_p(l*(l-1)+1,1) win_points_p(l*(l-1)+1,2); ...
                           win_points_p(l*l,1) win_points_p(l*l,2)];
    
    % Make sure coords are within bounds
    idx_inbounds = win_points_p(:,1) >= 1 & win_points_p(:,1) <= width & ...
                   win_points_p(:,2) >= 1 & win_points_p(:,2) <= height;
               
    % Only keep inbound idx
    win_points_p = win_points_p(idx_inbounds,:);        
    win_point_weights = win_point_weights(idx_inbounds,:);
    
    % TODO: Possibly update win_point_corners_p in the future if the window
    % is out of bounds
end

function l_p = window_lengths_p(point_w,homography,wf,calib_config)
    % Computes a window, in pixel coordinates, using input point in world
    % coordinates, homography, window_factor and calib_config, then calculates
    % the lengths of each side of the window.
    %
    %   Points and lengths are:
    %       p1 - l2 - p3
    %       |         |
    %       l1   p_w  l4
    %       |         |
    %       p2 - l3 - p4
    
    % Get points in world coordinates
    p1_w = [point_w(1)-(calib_config.square_size/2)*wf, ...
            point_w(2)-(calib_config.square_size/2)*wf];
    p2_w = [point_w(1)-(calib_config.square_size/2)*wf, ...
            point_w(2)+(calib_config.square_size/2)*wf];
    p3_w = [point_w(1)+(calib_config.square_size/2)*wf, ...
            point_w(2)-(calib_config.square_size/2)*wf];
    p4_w = [point_w(1)+(calib_config.square_size/2)*wf, ...
            point_w(2)+(calib_config.square_size/2)*wf];
        
    % Apply homography
    p_win_p = alg.apply_homography(homography,vertcat(p1_w,p2_w,p3_w,p4_w));
    
    % Calculate distances
    l_p(1) = norm(p_win_p(2,:)-p_win_p(1,:));
    l_p(2) = norm(p_win_p(3,:)-p_win_p(1,:));
    l_p(3) = norm(p_win_p(4,:)-p_win_p(2,:));
    l_p(4) = norm(p_win_p(4,:)-p_win_p(3,:));
end

function wf = window_factor(point_w,homography,calib_config)
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
    wf = calib_config.refine_corner_default_window_factor;
        
    % Get window lengths in pixel coordinates
    l_p = window_lengths_p(point_w, ...
                           homography, ...
                           wf, ...
                           calib_config);
        
    % Recompute window_factor if any of the distances are below the minimum
    % window size
    if any(l_p < calib_config.refine_corner_window_min_size)
        if calib_config.verbose > 2
            warning('min window constraint met; recomputing window factor for this corner.');
        end
        
        [~, min_idx] = min(l_p);
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
        r = roots([calib_config.refine_corner_window_min_size^2*f^2*e^2-(a*f-e*b)^2-(c*f-e*d)^2 ...
                   2*calib_config.refine_corner_window_min_size^2*f*e*(l*f+l*e)-2*(a*f-e*b)*(f*j+l*a-e*j-l*b)-2*(c*f-e*d)*(f*k+l*c-e*k-l*d) ...
                   2*calib_config.refine_corner_window_min_size^2*l^2*f*e+calib_config.refine_corner_window_min_size^2*(l*f+l*e)^2-(f*j+l*a-e*j-l*b)^2-(f*k+l*c-e*k-l*d)^2 ...
                   2*calib_config.refine_corner_window_min_size^2*l^2*(l*f+l*e) ...
                   calib_config.refine_corner_window_min_size^2*l^4]);

        % Get smallest, real, and positive root to get window_factor.
        wf = min(r(arrayfun(@(x)isreal(x(1)),r) & r > 0));
        wf = 2*wf/calib_config.square_size;
    end
    
    % Threshold window_factor to 4/3 to prevent overlap
    if wf >= 4/3
        if calib_config.verbose > 2
            warning('max window_factor is being set.');
        end
        wf = 4/3;
    end
end

function hw = half_window(point_w,homography,wf,calib_config)
    % Computes half window used for refinement window
        
    hw = floor(max(window_lengths_p(point_w,homography,wf,calib_config))/4)*2+1;
end

function win_points_p = window_points_p(point_w,homography,wf,hw,calib_config)
    % Computes window points in pixel coordinates
    
    % Get grid of points in world coordinates
    [win_points_y, win_points_x] = ndgrid(linspace(point_w(2)-(calib_config.square_size/2)*wf, ...
                                                   point_w(2)+(calib_config.square_size/2)*wf, ...
                                                   2*hw+1), ...
                                          linspace(point_w(1)-(calib_config.square_size/2)*wf, ...
                                                   point_w(1)+(calib_config.square_size/2)*wf, ...
                                                   2*hw+1));        
    win_points_w = [win_points_x(:) win_points_y(:)];        
    
    % Apply homography to window points to bring them into pixel
    % coordinates
    win_points_p = alg.apply_homography(homography,win_points_w);
end

function weights = window_point_weights(hw)
    % Computes weights of window points.
       
    % TODO: possibly add sigma parameter to cb config file
    
    % Get gaussian kernel, scale weights between 0 and 1 and return a vector
    weights = fspecial('Gaussian',[2*hw+1 2*hw+1],hw/2);
    weights = (weights-min(weights(:)))./(max(weights(:))-min(weights(:)));
    weights = reshape(weights,[],1);
end
