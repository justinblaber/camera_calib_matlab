function points_p = refine_points(points_p,cb_img,homography,cal_config)
    % This will return refined point coordinates. This uses the technique 
    % of getting the dot product, in a window, of the displacement vector 
    % and gradient, which should be zero for all points within the window.
    %
    % Inputs:
    %   points_p - array; Nx2 array of points in pixel coordinates
    %   cb_img - class.img; calibration board image
    %   homography - array; 3x3 homography matrix. This is used to compute
    %       the window around each point.
    %   cal_config - struct; this is the struct returned by
    %       util.load_cal_config()
    %
    % Outputs:
    %   points_p - array; Nx2 array of refined points.
        
    if cal_config.verbose > 1
        disp('---');
    end
    
    % Get calibration board image
    cb_gs = cb_img.get_gs();
        
    % Get gradient images
    cb_gs_dx = alg.array_grad(cb_gs,'x');
    cb_gs_dy = alg.array_grad(cb_gs,'y');
                                   
    % Perform iterations until convergence
    if cal_config.verbose > 1
        disp(['Refining points for: ' cb_img.get_path() '...']);
    end
    for i = 1:size(points_p,1)           
        for it = 1:cal_config.refine_corner_it_cutoff
            % Keep copy of point before updating it        
            point_prev = points_p(i,:);   
            
            % Get window points in pixel coordinates
            [win_points_p,win_point_weights] = alg.refine_window_p(points_p(i,:), ...
                                                                   homography, ...
                                                                   cb_img.get_width(), ...
                                                                   cb_img.get_height(), ...
                                                                   cal_config);
                
            % Refine point
            points_p(i,:) = refine_point_it(cb_gs_dx, ...
                                            cb_gs_dy, ...
                                            win_points_p, ...                                          
                                            win_point_weights);  
                                                  
            % Exit if change in distance is small
            diff_norm = norm(point_prev-points_p(i,:));
            if diff_norm < cal_config.refine_corner_norm_cutoff
                break
            end
        end  
        if cal_config.verbose > 2
            if it == cal_config.refine_corner_it_cutoff
                warning('iterations hit cutoff before converging!!!');
            else
                disp(['Iterations: ' num2str(it)]);
            end
            disp(['Pixel difference norm: ' num2str(diff_norm)]);
        end
    end  
end

function point_p = refine_point_it(cb_gs_dx,cb_gs_dy,win_points_p,win_point_weights)
    % Interpolate gradients
    cb_gs_dx_window = alg.array_interp(cb_gs_dx,win_points_p,'cubic');
    cb_gs_dy_window = alg.array_interp(cb_gs_dy,win_points_p,'cubic');
    
    % Apply weights
    cb_gs_dx_window = cb_gs_dx_window.*win_point_weights;
    cb_gs_dy_window = cb_gs_dy_window.*win_point_weights;

    % Form linear system
    A = [cb_gs_dx_window cb_gs_dy_window];
    b = (cb_gs_dx_window.*win_points_p(:,1) + cb_gs_dy_window.*win_points_p(:,2));

    % Solve for updated point
    point_p = (pinv(A)*b)';  
end
