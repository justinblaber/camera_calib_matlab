function points_p = refine_points(points_p,cb_img,homography,calib_config)
    % This will return refined point coordinates. This uses the technique 
    % of getting the dot product, in a window, of the displacement vector 
    % and gradient, which should be zero for all points within the window.
    %
    % Inputs:
    %   points_p - array; Nx2 array of points in pixel coordinates
    %   cb_img - class.img; calibration board image
    %   homography - array; 3x3 homography matrix. This is used to compute
    %       the window around each point.
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   points_p - array; Nx2 array of refined points.
        
    if calib_config.verbose > 1
        disp('---');
    end
    
    % Get calibration board image
    cb_gs = cb_img.get_gs();
        
    % Get gradient images
    cb_gs_dx = alg.array_grad(cb_gs,'x');
    cb_gs_dy = alg.array_grad(cb_gs,'y');
                                   
    % Perform iterations until convergence
    % Cache image dimensions for speed
    img_width = cb_img.get_width();
    img_height = cb_img.get_height();
    % Cache inverse homography for speed
    homography_inv = homography^-1;
    % Get interpolators for gradient images    
    I_cb_gs_dx = griddedInterpolant({1:size(cb_gs_dx,1),1:size(cb_gs_dx,2)}, ...
                                     cb_gs_dx,'cubic','none');
    I_cb_gs_dy = griddedInterpolant({1:size(cb_gs_dy,1),1:size(cb_gs_dy,2)}, ...
                                     cb_gs_dy,'cubic','none');
    for i = 1:size(points_p,1)           
        for it = 1:calib_config.refine_corner_it_cutoff
            % Keep copy of point before updating it        
            point_prev = points_p(i,:);   
            
            % Get window points in pixel coordinates
            [win_points_p,win_point_weights] = alg.refine_window_p(points_p(i,:), ...
                                                                   homography, ...
                                                                   homography_inv, ...
                                                                   img_width, ...
                                                                   img_height, ...
                                                                   calib_config);
                
            % Refine point
            point_p = refine_point_it(I_cb_gs_dx, ...
                                      I_cb_gs_dy, ...
                                      win_points_p, ...                                          
                                      win_point_weights);  
                                  
            % Make sure point is in bounds
            if point_p(:,1) < 1 || point_p(:,1) > img_width || ...
               point_p(:,2) < 1 || point_p(:,2) > img_height
                break
            end
                        
            % Store
            points_p(i,:) = point_p;
                                                  
            % Exit if change in distance is small
            diff_norm = norm(point_prev-points_p(i,:));
            if diff_norm < calib_config.refine_corner_norm_cutoff
                break
            end
        end  
        if calib_config.verbose > 2
            if it == calib_config.refine_corner_it_cutoff
                warning('iterations hit cutoff before converging!!!');
            else
                disp(['Iterations: ' num2str(it)]);
            end
            disp(['Pixel difference norm: ' num2str(diff_norm)]);
        end
    end  
end

function point_p = refine_point_it(I_cb_gs_dx,I_cb_gs_dy,win_points_p,win_point_weights)
    % Interpolate gradients
    cb_gs_dx_window = I_cb_gs_dx(win_points_p(:,2),win_points_p(:,1));
    cb_gs_dy_window = I_cb_gs_dy(win_points_p(:,2),win_points_p(:,1));
    
    % Apply weights
    cb_gs_dx_window = cb_gs_dx_window.*win_point_weights;
    cb_gs_dy_window = cb_gs_dy_window.*win_point_weights;

    % Form linear system
    A = [cb_gs_dx_window cb_gs_dy_window];
    b = (cb_gs_dx_window.*win_points_p(:,1) + cb_gs_dy_window.*win_points_p(:,2));

    % Solve for updated point
    point_p = mldivide(A,b)';  
end