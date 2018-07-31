function points_p = refine_checkers(points_p,array,homography,calib_config)
    % This will return refined point coordinates of checker(s). This uses 
    % the technique of getting the dot product, in a window, of the 
    % displacement vector and gradient, which should be zero for all points
    % within the window.
    %
    % Inputs:
    %   points_p - array; Px2 array of points in pixel coordinates
    %   array - array; MxN array containing checkers
    %   homography - array; 3x3 homography matrix. This is used to compute
    %       the window around each point.
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   points_p - array; Px2 array of refined points.
        
    % Get gradient images
    array_dx = alg.array_grad(array,'x');
    array_dy = alg.array_grad(array,'y');
                                   
    % Perform iterations until convergence
    % Cache inverse homography for speed
    homography_inv = homography^-1;
    % Get interpolators for gradient images    
    I_array_dx = griddedInterpolant({1:size(array_dx,1),1:size(array_dx,2)}, ...
                                     array_dx,'cubic','none');
    I_array_dy = griddedInterpolant({1:size(array_dy,1),1:size(array_dy,2)}, ...
                                     array_dy,'cubic','none');
    for i = 1:size(points_p,1)           
        for it = 1:calib_config.refine_checker_it_cutoff
            % Keep copy of point before updating it        
            point_prev = points_p(i,:);   
            
            % Get window points in pixel coordinates
            [win_points_p,win_point_weights] = alg.refine_checker_window(points_p(i,:), ...
                                                                         homography, ...
                                                                         homography_inv, ...
                                                                         size(array,2), ...
                                                                         size(array,1), ...
                                                                         calib_config);
                
            % Refine point
            point_p = refine_point_it(I_array_dx, ...
                                      I_array_dy, ...
                                      win_points_p, ...                                          
                                      win_point_weights);  
                                  
            % Make sure point is in bounds
            if point_p(:,1) < 1 || point_p(:,1) > size(array,2) || ...
               point_p(:,2) < 1 || point_p(:,2) > size(array,1)
                break
            end
                        
            % Store
            points_p(i,:) = point_p;
                                                  
            % Exit if change in distance is small
            diff_norm = norm(point_prev-points_p(i,:));
            if diff_norm < calib_config.refine_checker_norm_cutoff
                break
            end
        end  
        if it == calib_config.refine_checker_it_cutoff
            warning('iterations hit cutoff before converging!!!');
        end
    end  
end

function point_p = refine_point_it(I_array_dx,I_array_dy,win_points_p,win_point_weights)
    % Interpolate gradients
    array_dx_window = I_array_dx(win_points_p(:,2),win_points_p(:,1));
    array_dy_window = I_array_dy(win_points_p(:,2),win_points_p(:,1));
    
    % Apply weights
    array_dx_window = array_dx_window.*win_point_weights;
    array_dy_window = array_dy_window.*win_point_weights;

    % Form linear system
    A = [array_dx_window array_dy_window];
    b = (array_dx_window.*win_points_p(:,1) + array_dy_window.*win_points_p(:,2));

    % Solve for updated point
    point_p = mldivide(A,b)';  
end
