function points = refine_points(points,cb_img,homography,cb_config)
    % This will return refined point coordinates given an initial guess for
    % the points, a calibration board image, initial guess of the 
    % homography, and calibration board config. This uses the technique of 
    % getting the dot product, in a window, of the displacement vector and
    % gradient, which should be zero for all points within the window.
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
        for it = 1:cb_config.refine_corner_it_cutoff
            % Keep copy of point before updating it        
            point_prev = points(i,:);   
            
            % Get window points in image coordinates
            [win_points_i,win_point_weights] = alg.refine_window_i(points(i,:), ...
                                                                   homography, ...
                                                                   cb_img.get_width(), ...
                                                                   cb_img.get_height(), ...
                                                                   cb_config);
                
            % Refine point
            points(i,:) = refine_point_it(cb_gs_dx, ...
                                          cb_gs_dy, ...
                                          win_points_i, ...                                          
                                          win_point_weights);  
                                                  
            % Exit if change in distance is small
            diff_norm = norm(point_prev-points(i,:));
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