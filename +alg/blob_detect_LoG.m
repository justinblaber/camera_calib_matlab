function blobs = blob_detect_LoG(array,opts)
    % Performs blob detection on input array; this returns dark blobs.
    % 
    % Inputs:
    %   array - array; MxN array
    %   opts - struct;
    %       .blob_detect_r_range1 - scalar; starting point for blob radius
    %           search.
    %       .blob_detect_r_range2 - scalar; end point for blob radius
    %           search.
    %       .blob_detect_step - scalar; increment for blob radius search.
    %       .blob_detect_num_cutoff - int; max number of blobs found.
    %       .blob_detect_LoG_cutoff - scalar; cutoff for blob LoG value.
    %       .blob_detect_LoG_interp - string; interpolation used for
    %           LoG resampling.
    %       .blob_detect_eccentricity_cutoff - scalar; cutoff for
    %           eccentricity of blob.
    %       .blob_detect_maxima_it_cutoff - int; max number of iterations
    %           performed for refinement of maxima parameters.
    %       .blob_detect_maxima_norm_cutoff - scalar; cutoff for norm of
    %           difference of parameter vector for refinement of maxima
    %           parameters.
    %       .blob_detect_centroid_it_cutoff - int; max number of iterations
    %           performed for refinement of centroid.
    %       .blob_detect_centroid_norm_cutoff - scalar; cutoff for norm of
    %           difference of parameter vector for refinement of centroid.
    %       .blob_detect_r_it_cutoff - int; max number of iterations
    %           performed for refinement of radius.
    %       .blob_detect_r_norm_cutoff - scalar; cutoff for norm of
    %           difference of radius.
    %       .blob_detect_d_cluster - scalar; clusters blobs within d
    %           distance.
    %       .blob_detect_r1_cluster - scalar; clusters blobs within r1
    %           distance.
    %       .blob_detect_r2_cluster - scalar; clusters blobs within r2
    %           distance.
    %
    % Outputs:
    %   blobs - array; Px5 array containing:
    %       blobs(i,1) = h; x component of center of blob
    %       blobs(i,2) = k; y component of center of blob
    %       blobs(i,3) = a; major axis length 
    %       blobs(i,4) = b; minor axis length
    %       blobs(i,5) = alpha; rotation of major axis
        
    % Normalize array
    array = alg.normalize_array(array,'min-max');
    
    % Precompute gradients
    array_dx = alg.grad_array(array,'x');  
    array_dy = alg.grad_array(array,'y'); 
    
    % Get scale normalized LoG stack -------------------------------------%
    
    % Get number of scales
    r_range1 = opts.blob_detect_r_range1;  
    r_range2 = opts.blob_detect_r_range2;   
    step = opts.blob_detect_step;
    num_scales = ceil((r_range2-r_range1)/step) + 1;
            
    % Create scale normalized LoG stack
    stack_LoG = zeros([size(array) num_scales]);
    for i = 1:num_scales
        r = (i-1)*step + r_range1;                                        % radius
        sigma = r/sqrt(2);                                                % standard deviation
        kernel_LoG = sigma^2 * fspecial('log', 2*ceil(4*sigma)+1, sigma); % scale normalized LoG kernel  
    
        % Apply LoG filter
        stack_LoG(:,:,i) = imfilter(array, kernel_LoG, 'same', 'replicate');
    end 
    
    % Get interpolator
    I_stack_LoG = griddedInterpolant({1:size(stack_LoG,1),1:size(stack_LoG,2),1:size(stack_LoG,3)}, ...
                                      stack_LoG, ...
                                      opts.blob_detect_LoG_interp, ...
                                      'none');
            
    % Get initial maxima of scale normalized LoG stack -------------------%
    
    % Assign, to every voxel, the maximum of its neighbors. Then, see if 
    % voxel value is greater than this value; if true, it's a local maxima.
    kernel = true(3,3,3);
    kernel(2,2,2) = false;
    maxima = stack_LoG > imdilate(stack_LoG, kernel);

    % Clear out edge values
    maxima(  1,  :,  :) = false;
    maxima(end,  :,  :) = false;
    maxima(  :,  1,  :) = false;
    maxima(  :,end,  :) = false;
    maxima(  :,  :,  1) = false;
    maxima(  :,  :,end) = false;

    % Get initial coordinates of maxima
    idx_maxima_init = find(maxima);
    [y_maxima_init,x_maxima_init,idx_r_maxima_init] = ind2sub(size(maxima),idx_maxima_init);
    
    % Get most powerful blob responses specified by num_cutoff and
    % LoG_cutoff
    maxima_vals = stack_LoG(idx_maxima_init);
    [~,idx_maxima_sorted] = sort(maxima_vals,'descend');
    idx_maxima_sorted = idx_maxima_sorted(1:min(opts.blob_detect_num_cutoff,end));
    idx_maxima_sorted = idx_maxima_sorted(maxima_vals(idx_maxima_sorted) > opts.blob_detect_LoG_cutoff);
    x_maxima_init = x_maxima_init(idx_maxima_sorted);
    y_maxima_init = y_maxima_init(idx_maxima_sorted);
    idx_r_maxima_init = idx_r_maxima_init(idx_maxima_sorted);
    
    % Get refined maxima of scale normalized LoG stack -------------------%
                        
    % Initialize blobs
    blobs = zeros(0,5);
    
    % Initialize "box" to interpolate in order to compute finite difference
    % approximations to gradient/hessian
    [y_box,x_box,idx_r_box] = ndgrid(-1:1:1,-1:1:1,-1:1:1);
        
    % Iterate
    for i = 1:numel(x_maxima_init)
        % Grab initial values
        x_maxima = x_maxima_init(i);
        y_maxima = y_maxima_init(i);
        idx_r_maxima = idx_r_maxima_init(i);
        
        % Get sub-pixel values with gauss newton method ------------------%    
        
        % Iterate
        for it = 1:opts.blob_detect_maxima_it_cutoff
            % Get finite difference coordinates for interpolation
            x_fd = x_box + x_maxima;
            y_fd = y_box + y_maxima;
            idx_r_fd = idx_r_box + idx_r_maxima;
            
            % Check to make sure finite difference box is in range
            if any(x_fd(:) < 1 | x_fd(:) > size(stack_LoG,2) | ...
                   y_fd(:) < 1 | y_fd(:) > size(stack_LoG,1) | ...
                   idx_r_fd(:) < 1 | idx_r_fd(:) > size(stack_LoG,3))
                x_maxima = NaN;
                y_maxima = NaN;
                idx_r_maxima = NaN;
                break
            end
            
            % Interpolate
            LoG_fd = reshape(I_stack_LoG(y_fd(:),x_fd(:),idx_r_fd(:)),3,3,3);
            
            % Get gradient: 
            %   [dLoG/dx dLoG/dy dLoG/didx_r]
            grad = [(LoG_fd(2,3,2)-LoG_fd(2,1,2))/2;
                    (LoG_fd(3,2,2)-LoG_fd(1,2,2))/2; 
                    (LoG_fd(2,2,3)-LoG_fd(2,2,1))/2];

            % Get hessian:
            %   [d^2LoG/dx^2        d^2LoG/(dx*dy)      d^2LoG/(dx*didx_r)
            %    d^2LoG/(dy*dx)     d^2LoG/dy^2         d^2LoG/(dy*didx_r)
            %    d^2LoG/(didx_r*dx) d^2LoG/(didx_r*dy)  d^2LoG/(didx_r^2)] 
            hess = [LoG_fd(2,3,2)-2*LoG_fd(2,2,2)+LoG_fd(2,1,2),                         ((LoG_fd(3,3,2)-LoG_fd(1,3,2))/2-(LoG_fd(3,1,2)-LoG_fd(1,1,2))/2)/2, ((LoG_fd(2,3,3)-LoG_fd(2,3,1))/2-(LoG_fd(2,1,3)-LoG_fd(2,1,1))/2)/2;
                    ((LoG_fd(3,3,2)-LoG_fd(1,3,2))/2-(LoG_fd(3,1,2)-LoG_fd(1,1,2))/2)/2, LoG_fd(3,2,2)-2*LoG_fd(2,2,2)+LoG_fd(1,2,2),                         ((LoG_fd(3,2,3)-LoG_fd(3,2,1))/2-(LoG_fd(1,2,3)-LoG_fd(1,2,1))/2)/2;
                    ((LoG_fd(2,3,3)-LoG_fd(2,3,1))/2-(LoG_fd(2,1,3)-LoG_fd(2,1,1))/2)/2, ((LoG_fd(3,2,3)-LoG_fd(3,2,1))/2-(LoG_fd(1,2,3)-LoG_fd(1,2,1))/2)/2, LoG_fd(2,2,3)-2*LoG_fd(2,2,2)+LoG_fd(2,2,1)];

            % Get incremental parameters
            try
                delta_params = linsolve(-hess, grad, struct('POSDEF',true,'SYM',true));
            catch
                % This actually removes a lot of non-blob points
                x_maxima = NaN;
                y_maxima = NaN;
                idx_r_maxima = NaN;
                break
            end
            
            if any(delta_params > 1)
                % Limit maximum change to 1 since maxima should not move
                % too much
                delta_params = delta_params./max(delta_params);
            end
            
            % Update params
            x_maxima = x_maxima + delta_params(1);
            y_maxima = y_maxima + delta_params(2);
            idx_r_maxima = idx_r_maxima + delta_params(3);
            
            % Exit if change in distance is small
            diff_norm = norm(delta_params);            
            if diff_norm < opts.blob_detect_maxima_norm_cutoff
                break
            end
        end
        
        if isnan(x_maxima) || isnan(y_maxima) || isnan(idx_r_maxima)
            continue
        end
        
        % Get optimized values
        x = x_maxima;
        y = y_maxima;
        r = (idx_r_maxima-1)*step + r_range1;
                                
        % Get sub array containing blob ----------------------------------%
        
        % Get bounding box
        hw = ceil(4*r);
        bb_sub_array = [round(x)-hw round(y)-hw;
                        round(x)+hw round(y)+hw];
                    
        % Make sure sub array is in bounds
        if bb_sub_array(1,1) < 1 || bb_sub_array(2,1) > size(array,2) || ...
           bb_sub_array(1,2) < 1 || bb_sub_array(2,2) > size(array,1)
            continue
        end    
        
        % Grab sub arrays
        sub_array = array(bb_sub_array(1,2):bb_sub_array(2,2),bb_sub_array(1,1):bb_sub_array(2,1));
        sub_array_dx = array_dx(bb_sub_array(1,2):bb_sub_array(2,2),bb_sub_array(1,1):bb_sub_array(2,1));  
        sub_array_dy = array_dy(bb_sub_array(1,2):bb_sub_array(2,2),bb_sub_array(1,1):bb_sub_array(2,1));         
                        
        % Get sub_array coordinates        
        [y_sub_array,x_sub_array] = ndgrid(bb_sub_array(1,2):bb_sub_array(2,2), ...
                                           bb_sub_array(1,1):bb_sub_array(2,1));
                
        % Initialize ellipse ---------------------------------------------%
        
        e = [x y r r 0];
        
        % Refine ellipse with second moment matrix -----------------------%
        
        e = second_moment_ellipse(sub_array_dx, ...
                                  sub_array_dy, ...
                                  x_sub_array, ...
                                  y_sub_array, ...
                                  e, ...
                                  r);
        if e(3)/e(4) > opts.blob_detect_eccentricity_cutoff
            continue
        end

        % Refine center position of blob using centroid refinement -------%
        
        % Use inverse of sub array for centroid refinement
        sub_array_c = alg.normalize_array(-sub_array,'min-max');  
        
        % Iterate
        x = e(1);
        y = e(2);
        for it = 1:opts.blob_detect_centroid_it_cutoff
            % Store previous
            x_prev = x;
            y_prev = y;

            % Get weights
            W = weight_array(e,x_sub_array,y_sub_array);

            % Get refined x and y position
            sub_array_c_W = sub_array_c.*W;
            sum_sub_array_c_W = sum(sub_array_c_W(:));
            x = sum(sum(sub_array_c_W.*x_sub_array))/sum_sub_array_c_W;
            y = sum(sum(sub_array_c_W.*y_sub_array))/sum_sub_array_c_W;

            % Update center position of ellipse
            e(1) = x;
            e(2) = y;

            % Exit if change in distance is small
            diff_norm = sqrt((x_prev-x)^2+(y_prev-y)^2);        
            if diff_norm < opts.blob_detect_centroid_norm_cutoff
                break
            end
        end
             
        % Refine scale ---------------------------------------------------%
        
        % Convert r into index
        idx_r = (r-r_range1)/step+1;
        
        % Iterate
        for it = 1:opts.blob_detect_r_it_cutoff
            % Get finite difference coordinates for interpolation
            x_fd = [e(1) e(1) e(1)];
            y_fd = [e(2) e(2) e(2)];
            idx_r_fd = [idx_r-1 idx_r idx_r+1];

            % Check to make sure finite difference box is in range
            if any(x_fd(:) < 1 | x_fd(:) > size(stack_LoG,2) | ...
                   y_fd(:) < 1 | y_fd(:) > size(stack_LoG,1) | ...
                   idx_r_fd(:) < 1 | idx_r_fd(:) > size(stack_LoG,3))
                idx_r = NaN;
                break
            end

            % Interpolate
            LoG_fd = I_stack_LoG(y_fd(:),x_fd(:),idx_r_fd(:));

            % Get gradient: 
            %   [dLoG/didx_r]
            grad = (LoG_fd(3)-LoG_fd(1))/2;

            % Get hessian:
            %   [d^2LoG/(didx_r^2)] 
            hess = LoG_fd(3)-2*LoG_fd(2)+LoG_fd(1);

            % Get incremental parameters
            if hess < 0
                delta_params = -grad/hess;
            else
                idx_r = NaN;
                break
            end

            if any(delta_params > 1)
                % Limit maximum change to 1 since maxima should not move
                % too much
                delta_params = 1;
            end

            % Update params
            idx_r = idx_r + delta_params;

            % Exit if change in distance is small
            diff_norm = norm(delta_params);            
            if diff_norm < opts.blob_detect_r_norm_cutoff
                break
            end
        end

        if isnan(idx_r)
            continue
        end

        % Get new r 
        r = (idx_r-1)*step + r_range1;  
        
        % Refine ellipse with second moment matrix and updated scale -----%
        
        e = second_moment_ellipse(sub_array_dx, ...
                                  sub_array_dy, ...
                                  x_sub_array, ...
                                  y_sub_array, ...
                                  e, ...
                                  r);
        if e(3)/e(4) > opts.blob_detect_eccentricity_cutoff
            continue
        end
            
        % Re-check LoG value ---------------------------------------------% 
        
        % Convert r into index
        idx_r = (r-r_range1)/step+1;
        
        % Get value
        LoG_val = I_stack_LoG(e(2),e(1),idx_r);
        if isnan(LoG_val) || LoG_val <= opts.blob_detect_LoG_cutoff
            continue
        end        
                
        % Store blob -----------------------------------------------------%
        
        % Do some very rudimentary clustering
        dist_d = sqrt((blobs(:,1)-e(1)).^2 + (blobs(:,2)-e(2)).^2);
        dist_r1 = abs(blobs(:,3)-e(3));       
        dist_r2 = abs(blobs(:,4)-e(4));        
        if all(dist_d > opts.blob_detect_d_cluster | ...
               dist_r1 > opts.blob_detect_r1_cluster | ...
               dist_r2 > opts.blob_detect_r2_cluster)
            blobs(end+1,:)= e; %#ok<AGROW>
        end
    end
end

function W = weight_array(e,xs,ys)
    [cov, p] = alg.ellipse2cov(e);
    W = mvnpdf([xs(:) ys(:)], p, cov);
    W = reshape(W,size(xs));
end

function e = second_moment_ellipse(array_dx,array_dy,xs,ys,e,r)
    % Get weight matrix
    W = weight_array(e,xs,ys);

    % Get second moment matrix
    M = zeros(2);
    M(1,1) = sum(W(:).*array_dx(:).^2);
    M(1,2) = sum(W(:).*array_dx(:).*array_dy(:));
    M(2,2) = sum(W(:).*array_dy(:).^2);
    M(2,1) = M(1,2);    
       
    % Get shape of ellipse from second moment matrix
    e = alg.cov2ellipse(inv(M),e(1:2)');
    
    % Constrain so:
    %   minor axis + major axis = 2*radius
    sf = 2*r/(e(3)+e(4));
    e(3) = sf*e(3);
    e(4) = sf*e(4);
end
