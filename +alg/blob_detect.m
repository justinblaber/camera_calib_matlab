function blobs = blob_detect(array,opts)
    % Performs blob detection on input array. By default this returns dark
    % blobs.
    % 
    % Inputs:
    %   array - array; MxN array
    %   opts - struct;
    %       .blob_detect_r_range1 - ;
    %       .blob_detect_r_range2 - ;
    %       .blob_detect_step - ;
    %       .blob_detect_num_cutoff - ;
    %       .blob_detect_LoG_cutoff - ;
    %       .blob_detect_it_cutoff - ;
    %       .blob_detect_norm_cutoff - ;
    %       .blob_detect_eig_ratio_cutoff - ;
    %       .blob_detect_centroid_it_cutoff - ;
    %       .blob_detect_centroid_norm_cutoff - ;
    %       .blob_detect_M_it_cutoff - ;
    %       .blob_detect_d_cluster - ;
    %       .blob_detect_r_cluster - ;
    %
    % Outputs:
    %   blobs - struct; contains:
    %       .x - scalar; x location of blob in pixels
    %       .y - scalar; y location of blob in pixels
    %       .r - scalar; radius of blob in pixels
    %       .M - array; second moment matrix
        
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
            
    % Create scale normalized LoG stack
    stack_LoG = zeros([size(array) num_scales]);
    for i = 1:num_scales
        r = (i-1)*step + r_range1;                                        % radius
        sigma = r/sqrt(2);                                                % standard deviation
        kernel_LoG = sigma^2 * fspecial('log', 2*ceil(4*sigma)+1, sigma); % scale normalized LoG kernel  
    
        % Apply LoG filter
        stack_LoG(:,:,i) = imfilter(array, kernel_LoG, 'same', 'replicate');
    end 
        
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
    % approximations to gradient/hessian and also get interpolator
    [y_box,x_box,idx_r_box] = ndgrid(-1:1:1,-1:1:1,-1:1:1);
    I_stack_LoG = griddedInterpolant({1:size(stack_LoG,1),1:size(stack_LoG,2),1:size(stack_LoG,3)}, ...
                                      stack_LoG,'spline','none');
    
    % Initialize blobs struct
    blobs = struct('x',{},'y',{},'r',{},'M',{});    
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
            %   [dLoG/dx dLoG/dy dLoG/dr_idx]
            grad = [(LoG_fd(2,3,2)-LoG_fd(2,1,2))/2;
                    (LoG_fd(3,2,2)-LoG_fd(1,2,2))/2; 
                    (LoG_fd(2,2,3)-LoG_fd(2,2,1))/2];

            % Get hessian:
            %   [d^2LoG/dx^2         d^2LoG/(dx*dy)     d^2LoG/(dx*dr_idx)
            %    d^2LoG/(dy*dx)     d^2LoG/dy^2         d^2LoG/(dy*dr_idx)
            %    d^2LoG/(dr_idx*dx) d^2LoG/(dr_idx*dy)  d^2LoG/(dr_idx^2)] 
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
            idx_r_maxima = idx_r_maxima+delta_params(3);
            
            % Exit if change in distance is small
            diff_norm = norm(delta_params);            
            if diff_norm < opts.blob_detect_norm_cutoff
                break
            end
        end
        
        if isnan(x_maxima) || isnan(y_maxima) || isnan(idx_r_maxima)
            continue
        end
        
        % Set maxima_r
        r_maxima = step*(idx_r_maxima-1)+r_range1;
        
        %{
        % Next, get sub array containing blob
        half_window = ceil(4*r_maxima);
        sub_array_l = round(x_maxima)-half_window;
        sub_array_r = round(x_maxima)+half_window;   
        sub_array_t = round(y_maxima)-half_window;
        sub_array_b = round(y_maxima)+half_window;     
        if sub_array_l < 1 || sub_array_r > size(array,2) || ...
           sub_array_t < 1 || sub_array_b > size(array,1)
            continue
        end    
        sub_array = array(sub_array_t:sub_array_b,sub_array_l:sub_array_r);  
        sub_array_dx = array_dx(sub_array_t:sub_array_b,sub_array_l:sub_array_r);  
        sub_array_dy = array_dy(sub_array_t:sub_array_b,sub_array_l:sub_array_r);         
        
        % Get sub_array coordinates        
        [y_sub_array,x_sub_array] = ndgrid(sub_array_t:sub_array_b, ...
                                           sub_array_l:sub_array_r);
        
        % Compute second moment matrix to threshold edge response
        eig_ratio = second_moment_params(sub_array_dx, ...
                                         sub_array_dy, ...
                                         x_sub_array, ...
                                         y_sub_array, ...
                                         x_maxima, ...
                                         y_maxima, ...
                                         r_maxima, ...
                                         r_maxima, ...
                                         r_maxima, ... 
                                         0);
                                                        
        % Filter out edge response
        if isnan(eig_ratio) || eig_ratio > opts.blob_detect_eig_ratio_cutoff
            continue
        end
                                       
        % Refine center position of blob using centroid analysis
        % First, get "centroid array" which is negative of sub array; 
        % scaled between 0 and 1
        c_sub_array = -sub_array;
        c_sub_array = (c_sub_array-min(c_sub_array(:)))/(max(c_sub_array(:))-min(c_sub_array(:)));         
        for it = 1:opts.blob_detect_centroid_it_cutoff
            % Store previous
            maxima_x_prev = x_maxima;
            maxima_y_prev = y_maxima;
        
            % Get gaussian kernel
            kernel_gauss = mvnpdf([x_sub_array(:) y_sub_array(:)], ...
                                  [x_maxima y_maxima], ...
                                  [r_maxima^2 0; 0 r_maxima^2]);
            kernel_gauss = reshape(kernel_gauss,size(sub_array));

            % Get refined x and y position
            c_sub_array_weighted = c_sub_array.*kernel_gauss;
            c_sub_array_weighted_sum = sum(c_sub_array_weighted(:));
            x_maxima = sum(sum(c_sub_array_weighted.*x_sub_array))/c_sub_array_weighted_sum;
            y_maxima = sum(sum(c_sub_array_weighted.*y_sub_array))/c_sub_array_weighted_sum;

            diff_norm = sqrt((maxima_x_prev-x_maxima)^2 + (maxima_y_prev-y_maxima)^2);        
            if diff_norm < opts.blob_detect_centroid_norm_cutoff
                break
            end
        end
                               
        % Get iteratively refined second moment matrix.
        r1_ellipse = r_maxima;
        r2_ellipse = r_maxima;
        rot = 0;
        for it = 1:opts.blob_detect_M_it_cutoff          
            [eig_ratio,r1_ellipse,r2_ellipse,rot,M] = second_moment_params(sub_array_dx, ...
                                                                           sub_array_dy, ...
                                                                           x_sub_array, ...
                                                                           y_sub_array, ...
                                                                           x_maxima, ...
                                                                           y_maxima, ...
                                                                           r_maxima, ...
                                                                           r1_ellipse, ...
                                                                           r2_ellipse, ... 
                                                                           rot);   
            if isnan(eig_ratio) || eig_ratio > opts.blob_detect_eig_ratio_cutoff
                break
            end
        end              
              
        % Filter out edge response again
        if isnan(eig_ratio) || eig_ratio > opts.blob_detect_eig_ratio_cutoff
            continue
        end
        
        % Remove any out of range idx
        if idx_r_maxima < 1 || idx_r_maxima > num_scales || ...
           x_maxima < 1 || x_maxima > size(array,2) || ...
           y_maxima < 1 || y_maxima > size(array,1)
            continue
        end
        %}
        
        % Before storing blob, make sure there isn't another blob near
        % this one. This is ad-hoc clustering but is typically "good enough"
        dist_d = sqrt(([blobs.x]-x_maxima).^2 + ...
                      ([blobs.y]-y_maxima).^2);
        dist_r = abs([blobs.r]-r_maxima);        
        if all(dist_d > opts.blob_detect_d_cluster | ...
               dist_r > opts.blob_detect_r_cluster)
            blobs(end+1).x = x_maxima; %#ok<AGROW>
            blobs(end).y = y_maxima;
            blobs(end).r = r_maxima;
            % blobs(end).M = M;
        end
    end
end

function [eig_ratio,r1,r2,rot,M] = second_moment_params(array_dx,array_dy,x_array,y_array,x,y,r,r1,r2,rot)
    % Get gaussian kernel in the shape of the ellipse for weights
    rotation = [cos(rot) -sin(rot);  ...
                sin(rot)  cos(rot)];
    variance = [r1^2 0; ...
                0    r2^2];
    covariance = rotation*variance*rotation';              
    if rank(covariance) < 2
        eig_ratio = nan;
        r1 = nan;
        r2 = nan;
        rot = nan;
        M = nan(2);
        return
    end        

    % Get second moment matrix
    kernel_gauss = mvnpdf([x_array(:) y_array(:)], ...
                          [x y], ....
                          covariance);
    kernel_gauss = reshape(kernel_gauss,size(array_dx));
    M(1,1) = sum(kernel_gauss(:).*array_dx(:).^2);
    M(1,2) = sum(kernel_gauss(:).*array_dx(:).*array_dy(:));
    M(2,2) = sum(kernel_gauss(:).*array_dy(:).^2);
    M(2,1) = M(1,2);        

    % Get initial guess of ellipse by using second moment matrix
    % Get major/minor axis and rotation of ellipse
    [V,D] = eig(M);
    [D,idx_sorted] = sort(diag(D));
    V = V(:,idx_sorted);     
    % eigenvalue ratio
    eig_ratio = D(2)/D(1);
    % ellipse sides are proportional to sqrt of eigenvalues
    ellipse_scale_factor = 2*r/(sqrt(D(1))+sqrt(D(2))); % have minor and major axis sum to diameter of blob
    r1 = sqrt(D(2))*ellipse_scale_factor; % major axis radius
    r2 = sqrt(D(1))*ellipse_scale_factor; % minor axis radius        
    % rotation of major axis
    rot = -atan2(V(1,2),V(2,2));  
end
