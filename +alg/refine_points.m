function points_i = refine_points(points_i,cb_img,homography,cb_config)
    % This will return refined point coordinates. This uses the technique 
    % of getting the dot product, in a window, of the displacement vector 
    % and gradient, which should be zero for all points within the window.
    %
    % Inputs:
    %   points_i - array; Nx2 array of points in image coordinates
    %   cb_img - class.img; calibration board image
    %   homography - array; 3x3 homography matrix. This is used to compute
    %       the window around each point.
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   points_i - array; Nx2 array of refined points.
        
    % Get calibration board image
    cb_gs = cb_img.get_gs();
        
    % Get gradient images
    cb_gs_dx = alg.array_grad(cb_gs,'x');
    cb_gs_dy = alg.array_grad(cb_gs,'y');
                                   
    % Perform iterations until convergence
    disp(['Refining points for: ' cb_img.get_path() '...']);
    for i = 1:size(points_i,1)           
        for it = 1:cb_config.refine_corner_it_cutoff
            % Keep copy of point before updating it        
            point_prev = points_i(i,:);   
            
            % Get window points in image coordinates
            [win_points_i,win_point_weights] = alg.refine_window_i(points_i(i,:), ...
                                                                   homography, ...
                                                                   cb_img.get_width(), ...
                                                                   cb_img.get_height(), ...
                                                                   cb_config);
                
            % Refine point
            points_i(i,:) = refine_point_it(cb_gs_dx, ...
                                            cb_gs_dy, ...
                                            win_points_i, ...                                          
                                            win_point_weights);  
                                                  
            % Exit if change in distance is small
            diff_norm = norm(point_prev-points_i(i,:));
            if diff_norm < cb_config.refine_corner_norm_cutoff
                break
            end
        end  
        if it == cb_config.refine_corner_it_cutoff
            disp('WARNING: iterations hit cutoff before converging!!!');
        else
            disp(['Iterations: ' num2str(it)]);
        end
        disp(['Pixel difference norm: ' num2str(diff_norm)]);
    end    
    disp('--------------------------------------------');
end

function point_i = refine_point_it(cb_gs_dx,cb_gs_dy,win_points_i,win_point_weights)
    % Interpolate gradients
    cb_gs_dx_window = alg.array_interp(cb_gs_dx,win_points_i,'cubic');
    cb_gs_dy_window = alg.array_interp(cb_gs_dy,win_points_i,'cubic');
    
    % Apply weights
    cb_gs_dx_window = cb_gs_dx_window.*win_point_weights;
    cb_gs_dy_window = cb_gs_dy_window.*win_point_weights;

    % Form linear system
    A = [cb_gs_dx_window cb_gs_dy_window];
    b = (cb_gs_dx_window.*win_points_i(:,1) + cb_gs_dy_window.*win_points_i(:,2));

    % Solve for updated point
    point_i = (pinv(A)*b)';  
end