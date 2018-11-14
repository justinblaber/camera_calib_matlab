function blobs = blob_detect(array,opts)
    % Performs blob detection on input array. By default this returns dark
    % blobs.
    % 
    % Inputs:
    %   array - array; MxN array
    %   opts - struct;
    %
    % Outputs:
    %   blobs - struct; contains:
        
    % Normalize array
    array = alg.normalize_array(array);
    
    % Precompute gradients
    array_dx = alg.grad_array(array,'x');  
    array_dy = alg.grad_array(array,'y'); 
    
    % Get scale normalized LoG stack -------------------------------------%
    
    % Get number of scales
    r_range1 = opts.blob_detect_r_range1;  
    r_range2 = opts.blob_detect_r_range2;   
    step = opts.blob_detect_step;
    num_scales = ceil((r_range2-r_range1)/step) + 1;
        
    tic
    
    % Create scale normalized LoG stack
    stack_LoG = zeros([size(array) num_scales]);
    for i = 1:num_scales
        r = (i-1)*step + r_range1;                                        % radius
        sigma = r/sqrt(2);                                                % standard deviation
        kernel_LoG = sigma^2 * fspecial('log', 2*ceil(4*sigma)+1, sigma); % scale normalized LoG kernel  
    
        % Apply LoG filter
        stack_LoG(:,:,i) = imfilter(array, kernel_LoG, 'same', 'replicate');
    end 
    
    toc
    tic   
        
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
              
    % Initialize "box" to interpolate in order to compute finite difference
    % approximations to gradient/hessian
    [y_box,x_box,idx_r_box] = ndgrid(-1:1:1,-1:1:1,-1:1:1);
    
    % Get interpolator
    I_stack_LoG = griddedInterpolant({1:size(stack_LoG,1),1:size(stack_LoG,2),1:size(stack_LoG,3)}, ...
                                      stack_LoG,opts.blob_detect_interp,'none');
    
    % Initialize blobs struct
    blobs = struct('x',{},'y',{},'r1',{},'r2',{},'rot',{});    
    for i = 1:numel(x_maxima_init)
        % Grab initial values
        x_maxima = x_maxima_init(i);
        y_maxima = y_maxima_init(i);
        idx_r_maxima = idx_r_maxima_init(i);
        
        % Get sub-pixel values with gauss newton method ------------------%    
        
        for it = 1:opts.blob_detect_it_cutoff
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
            if diff_norm < opts.blob_detect_norm_cutoff
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
        
        % For each iteration:
        %   1) Refine x,y position with centroid analysis
        %   2) For new, position, find optimal scale
        %   3) Exit if point converges
        sub_array_inv = alg.normalize_array(-sub_array);  
        e = [x y r r 0];
        for j = 1:5
            % Update ellipse estimation using second moment matrix
            e = second_moment_ellipse(sub_array_dx, ...
                                      sub_array_dy, ...
                                      e, ...
                                      x_sub_array, ...
                                      y_sub_array, ...
                                      r);
            if e(3)/e(4) > opts.blob_detect_eccentricity_cutoff
                e(:) = NaN;
                break
            end

            % Refine center position of blob using centroid analysis
            for it = 1:opts.blob_detect_centroid_it_cutoff
                % Store previous
                x_prev = x;
                y_prev = y;
    
                % Get weights
                W = weight_array(e, x_sub_array,  y_sub_array);

                % Get refined x and y position
                sub_array_inv_W = sub_array_inv.*W;
                sum_sub_array_inv_W = sum(sub_array_inv_W(:));
                x = sum(sum(sub_array_inv_W.*x_sub_array))/sum_sub_array_inv_W;
                y = sum(sum(sub_array_inv_W.*y_sub_array))/sum_sub_array_inv_W;

                % Update e
                e(1) = x;
                e(2) = y;
                e = second_moment_ellipse(sub_array_dx, ...
                                          sub_array_dy, ...
                                          e, ...
                                          x_sub_array, ...
                                          y_sub_array, ...
                                          r);
                if e(3)/e(4) > opts.blob_detect_eccentricity_cutoff
                    e(:) = NaN;
                    break
                end
                
                diff_norm = sqrt((x_prev-x)^2+(y_prev-y)^2);        
                if diff_norm < opts.blob_detect_centroid_norm_cutoff
                    break
                end
            end
            
            if any(isnan(e))
                break
            end
            
            % Get new scale
            idx_r = (r-r_range1)/step+1;
            for it = 1:opts.blob_detect_it_cutoff
                % Get finite difference coordinates for interpolation
                x_fd = [x x x];
                y_fd = [y y y];
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
                try
                    delta_params = linsolve(-hess, grad, struct('POSDEF',true,'SYM',true));
                catch
                    % This actually removes a lot of non-blob points
                    idx_r = NaN;
                    break
                end

                if any(delta_params > 1)
                    % Limit maximum change to 1 since maxima should not move
                    % too much
                    delta_params = delta_params./max(delta_params);
                end

                % Update params
                idx_r = idx_r + delta_params;

                % Exit if change in distance is small
                diff_norm = norm(delta_params);            
                if diff_norm < opts.blob_detect_norm_cutoff
                    break
                end
            end
            
            if isnan(idx_r)
                e(:) = NaN;
                break
            end
            
            % Get new r 
            r = (idx_r-1)*step + r_range1;

            %{

            % Get iteratively refined second moment matrix -------------------%

            % Initialize e for sub array
            e_sub = [x-bb_sub_array(1,1)+1 y-bb_sub_array(1,2)+1 r r 0];

            % Iterate
            for it = 1:opts.blob_detect_M_it_cutoff          
                % Compute second moment matrix to threshold edge response
                e_sub = second_moment_ellipse(sub_array_dx, ...
                                              sub_array_dy, ...
                                              r, ...
                                              e_sub);

                % Filter out edge response
                if e_sub(3)/e_sub(4) > opts.blob_detect_eccentricity_cutoff
                    break
                end
            end              

            % Filter out edge response again
            if e_sub(3)/e_sub(4) > opts.blob_detect_eccentricity_cutoff
                continue
            end

            % Remove any out of range idx
            if idx_r_maxima < 1 || idx_r_maxima > num_scales || ...
               x_maxima < 1 || x_maxima > size(array,2) || ...
               y_maxima < 1 || y_maxima > size(array,1)
                continue
            end

            % Store ellipse wrt full array
            e = e_sub;
            e(1) = e(1)+bb_sub_array(1,1)-1;
            e(2) = e(2)+bb_sub_array(1,2)-1;
            %}
        end
        
        if any(isnan(e))
            continue
        end
            
        % Store blob -----------------------------------------------------%
        
        % Do some very rudimentary clustering
        dist_d = sqrt(([blobs.x]-e(1)).^2 + ...
                      ([blobs.y]-e(2)).^2);
        dist_r1 = abs([blobs.r1]-e(3));       
        dist_r2 = abs([blobs.r2]-e(4));        
        if all(dist_d > opts.blob_detect_d_cluster | ...
               dist_r1 > opts.blob_detect_r1_cluster | ...
               dist_r2 > opts.blob_detect_r2_cluster)
            blobs(end+1).x = e(1); %#ok<AGROW>
            blobs(end).y = e(2);
            blobs(end).r1 = e(3);
            blobs(end).r2 = e(4);
            blobs(end).rot = e(5);
        end
    end
    
    toc
end

function W = weight_array(e,x,y)
    [cov, p] = alg.ellipse2cov(e);
    W = mvnpdf([x(:) y(:)], p, cov);
    W = reshape(W,size(x));
end

function M = second_moment(array_dx,array_dy,W)
    % Get second moment matrix
    M(1,1) = sum(W(:).*array_dx(:).^2);
    M(1,2) = sum(W(:).*array_dx(:).*array_dy(:));
    M(2,2) = sum(W(:).*array_dy(:).^2);
    M(2,1) = M(1,2);     
end

function e = M2ellipse(M,p,r)
    % Get shape of ellipse from second moment matrix
    e = alg.cov2ellipse(M^(-1/2),p);
    
    % Constrain so: minor axis + major axis = 3*radius
    sf = 2*r/(e(3)+e(4));
    e(3) = sf*e(3);
    e(4) = sf*e(4);
end

function e = second_moment_ellipse(array_dx,array_dy,e,x,y,r)
    % Get weight matrix
    W = weight_array(e,x,y);

    % Get second moment matrix
    M = second_moment(array_dx,array_dy,W);
    
    % Estimate ellipse from second moment matrix
    e = M2ellipse(M,e(1:2)',r);
end
