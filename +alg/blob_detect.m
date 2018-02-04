function blobs = blob_detect(array,calib_config)
    % Performs blob detection. By default this returns dark blobs.
    % 
    % Inputs:
    %   array - array; MxN array
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   blobs - struct; contains:
    %       .x - scalar; x location of blob in pixels
    %       .y - scalar; y location of blob in pixels
    %       .r - scalar; radius of blob in pixels
    %       .M - array; second moment matrix
    
    % TODO: for the defaults, you can, instead of setting them here, set 
    % them in a wrapper function to make this function more general...
    
    % Set radius ranges
    r_range1 = calib_config.blob_detect_r_range1;    
    if calib_config.blob_detect_r_range2 == realmax % This is the flag to set default
        % Markers should be relatively small
        r_range2 = min(size(array))/40;
    else
        r_range2 = calib_config.blob_detect_r_range2;
    end  
    
    % Make sure r_range2 is an integer number of steps away from r_range1
    num_scales = ceil((r_range2-r_range1)/calib_config.blob_detect_step+1);
    r_range2 = r_range1 + (num_scales-1)*calib_config.blob_detect_step;
            
    if r_range2 <= r_range1
        error(['r_range2: ' num2str(r_range2) ' is less than or equal to ' ...
               'r_range1: ' num2str(r_range1) '.']);
    end
    
    % Set num_cutoff
    if calib_config.blob_detect_num_cutoff == realmax % This is the flag to set default
        % 4 markers + ~number of squares + 500 blob cushion
        num_cutoff = 4 + calib_config.num_squares_height*calib_config.num_squares_width + 500;            
    else
        num_cutoff = calib_config.blob_detect_num_cutoff;
    end
    
    % Normalize array - this ensures thresholds and stuff work consistently
    array = (array-min(array(:)))/(max(array(:))-min(array(:)));
    
    % Create scale normalized LoG stack
    stack_LoG = zeros([size(array) num_scales]);
    for i = 1:num_scales
        sigma = (calib_config.blob_detect_step*(i-1)+r_range1)/sqrt(2);
        window = 2*ceil(4*sigma)+1; % Dimensions must be odd 
        kernel_LoG = sigma^2*fspecial('log',window,sigma);    
    
        % Apply LoG filter
        stack_LoG(:,:,i) = imfilter(array,kernel_LoG,'same','replicate');
    end    
    
    % Get maxima: assign, to every voxel, the maximum of its neighbors. 
    % Then, see if voxel value is greater than this value; if this is true, 
    % then it's a local maxima (technique is from Jonas on stackoverflow). 
    % Note: finding maxima will return DARK blobs.
    kernel = true(3,3,3);
    kernel(2,2,2) = false;
    maxima = stack_LoG > imdilate(stack_LoG,kernel);

    % Clear out edge values
    maxima(  1,  :,  :) = false;
    maxima(end,  :,  :) = false;
    maxima(  :,  1,  :) = false;
    maxima(  :,end,  :) = false;
    maxima(  :,  :,  1) = false;
    maxima(  :,  :,end) = false;

    % Get initial coordinates of maxima
    maxima_idx = find(maxima);
    [maxima_y_init,maxima_x_init,maxima_r_idx_init] = ind2sub(size(maxima),maxima_idx);
    
    % Get most powerful blob responses specified by num_cutoff and
    % log_cutoff
    maxima_vals = stack_LoG(maxima_idx);
    [~,maxima_sorted_idx] = sort(maxima_vals,'descend');
    maxima_sorted_idx = maxima_sorted_idx(1:min(num_cutoff,end));
    maxima_sorted_idx = maxima_sorted_idx(maxima_vals(maxima_sorted_idx) > calib_config.blob_detect_LoG_cutoff);
    maxima_x_init = maxima_x_init(maxima_sorted_idx);
    maxima_y_init = maxima_y_init(maxima_sorted_idx);
    maxima_r_idx_init = maxima_r_idx_init(maxima_sorted_idx);
    
    % Perform "refinement" of blobs (subpixel location, clustering, 
    % thresholding, etc...).   
    blobs = struct('x',{},'y',{},'r',{},'M',{});    
    % Precompute gradients
    array_dx = alg.array_grad(array,'x');  
    array_dy = alg.array_grad(array,'y');     
    % Initialize "box" to interpolate in order to compute finite difference
    % approximations to gradient/hessian and also get interpolator
    [y_box,x_box,r_idx_box] = ndgrid(-1:1:1,-1:1:1,-1:1:1);
    I_stack_LoG = griddedInterpolant({1:size(stack_LoG,1),1:size(stack_LoG,2),1:size(stack_LoG,3)}, ...
                                      stack_LoG,'cubic','none');
    for i = 1:length(maxima_x_init)
        % Grab initial values
        maxima_x = maxima_x_init(i);
        maxima_y = maxima_y_init(i);
        maxima_r_idx = maxima_r_idx_init(i);
    
        % Perform gauss newton iterations until convergence; during 
        % iterations, set invalid points to NaNs
        for it = 1:calib_config.blob_detect_it_cutoff
            % Get finite difference coordinates for interpolation
            x_fd = x_box + maxima_x;
            y_fd = y_box + maxima_y;
            r_idx_fd = r_idx_box + maxima_r_idx;
            
            % Check to make sure finite difference box is in range
            if any(x_fd(:) < 1 | x_fd(:) > size(stack_LoG,2) | ...
                   y_fd(:) < 1 | y_fd(:) > size(stack_LoG,1) | ...
                   r_idx_fd(:) < 1 | r_idx_fd(:) > size(stack_LoG,3))
                maxima_x = NaN;
                maxima_y = NaN;
                maxima_r_idx = NaN;
                break
            end
            
            % Interpolate
            LoG_fd = reshape(I_stack_LoG(y_fd(:),x_fd(:),r_idx_fd(:)),3,3,3);
            
            % Get gradient: [d_log/d_x d_log/d_y d_log/d_r_idx]
            dl_dp(1) = (LoG_fd(2,3,2)-LoG_fd(2,1,2))/2;
            dl_dp(2) = (LoG_fd(3,2,2)-LoG_fd(1,2,2))/2; 
            dl_dp(3) = (LoG_fd(2,2,3)-LoG_fd(2,2,1))/2; 

            % Get hessian:
            %   [d^2_log/d_x^2         d^2_log/(d_x*d_y)     d^2_log/(d_x*d_r_idx)
            %    d^2_log/(d_y*d_x)     d^2_log/d_y^2         d^2_log/(d_y*d_r_idx)
            %    d^2_log/(d_r_idx*d_x) d^2_log/(d_r_idx*d_y) d^2_log/(d_r_idx^2)]                
            ddl_ddp(1,1) = LoG_fd(2,3,2) - 2*LoG_fd(2,2,2) + LoG_fd(2,1,2);
            ddl_ddp(1,2) = ((LoG_fd(3,3,2)-LoG_fd(1,3,2))/2 - (LoG_fd(3,1,2)-LoG_fd(1,1,2))/2)/2;
            ddl_ddp(1,3) = ((LoG_fd(2,3,3)-LoG_fd(2,3,1))/2 - (LoG_fd(2,1,3)-LoG_fd(2,1,1))/2)/2;      
            ddl_ddp(2,2) = LoG_fd(3,2,2) - 2*LoG_fd(2,2,2) + LoG_fd(1,2,2);
            ddl_ddp(2,3) = ((LoG_fd(3,2,3)-LoG_fd(3,2,1))/2 - (LoG_fd(1,2,3)-LoG_fd(1,2,1))/2)/2;
            ddl_ddp(3,3) = LoG_fd(2,2,3) - 2*LoG_fd(2,2,2) + LoG_fd(2,2,1);

            % Fill lower half of hessian
            ddl_ddp(2,1) = ddl_ddp(1,2);
            ddl_ddp(3,1) = ddl_ddp(1,3);
            ddl_ddp(3,2) = ddl_ddp(2,3);  

            % Find incremental parameters
            try
                opts.POSDEF = true; 
                opts.SYM = true; 
                delta_p = linsolve(-ddl_ddp,dl_dp',opts);
            catch
                % This actually removes a lot of non-blob points
                maxima_x = NaN;
                maxima_y = NaN;
                maxima_r_idx = NaN;
                break
            end
            
            if any(delta_p > 1)
                % Limit maximum change to 1 since maxima should not move
                % too much
                delta_p = delta_p./max(delta_p);
            end
            
            % Get optimized locations
            maxima_x = maxima_x+delta_p(1);
            maxima_y = maxima_y+delta_p(2);
            maxima_r_idx = maxima_r_idx+delta_p(3);
            
            % Exit if change in distance is small
            diff_norm = norm(delta_p);            
            if calib_config.verbose > 2
                disp(['Blob detect iteration #: ' num2str(it)]);
                disp(['Difference norm for nonlinear parameter refinement: ' num2str(diff_norm)]);
            end
            if diff_norm < calib_config.blob_detect_norm_cutoff
                break
            end
        end
        if calib_config.verbose > 2 && it == calib_config.blob_detect_it_cutoff
            warning('Blob detect iterations hit cutoff before converging!!!');
        end
        
        if isnan(maxima_x) || isnan(maxima_y) || isnan(maxima_r_idx)
            continue
        end
        
        % Set maxima_r
        maxima_r = (r_range2-r_range1)/(num_scales-1)*(maxima_r_idx-1)+r_range1;
        
        % Next, get sub array containing blob
        half_window = ceil(4*maxima_r);
        sub_array_l = round(maxima_x)-half_window;
        sub_array_r = round(maxima_x)+half_window;   
        sub_array_t = round(maxima_y)-half_window;
        sub_array_b = round(maxima_y)+half_window;     
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
                                         maxima_x, ...
                                         maxima_y, ...
                                         maxima_r, ...
                                         maxima_r, ...
                                         maxima_r, ... 
                                         0);
                                                        
        % Filter out edge response
        if isnan(eig_ratio) || eig_ratio > calib_config.blob_detect_eig_ratio_cutoff
            continue
        end
                                       
        % Refine center position of blob using centroid analysis
        % First, get "centroid array" which is negative of sub array, 
        % scaled between 0 and 1
        c_sub_array = -sub_array;
        c_sub_array = (c_sub_array-min(c_sub_array(:)))/(max(c_sub_array(:))-min(c_sub_array(:)));         
        for it = 1:calib_config.blob_detect_centroid_it_cutoff
            % Store previous
            maxima_x_prev = maxima_x;
            maxima_y_prev = maxima_y;
        
            % Get gaussian kernel
            kernel_gauss = mvnpdf([x_sub_array(:) y_sub_array(:)], ...
                                  [maxima_x maxima_y], ...
                                  [maxima_r^2 0; 0 maxima_r^2]);
            kernel_gauss = reshape(kernel_gauss,size(sub_array));

            % Get refined x and y position
            c_sub_array_weighted = c_sub_array.*kernel_gauss;
            c_sub_array_weighted_sum = sum(c_sub_array_weighted(:));
            maxima_x = sum(sum(c_sub_array_weighted.*x_sub_array))/c_sub_array_weighted_sum;
            maxima_y = sum(sum(c_sub_array_weighted.*y_sub_array))/c_sub_array_weighted_sum;

            diff_norm = sqrt((maxima_x_prev-maxima_x)^2 + (maxima_y_prev-maxima_y)^2);        
            if calib_config.verbose > 2
                disp(['Blob detect centroid refinement iteration #: ' num2str(it)]);
                disp(['Difference norm for centroid refinement: ' num2str(diff_norm)]);
            end
            if diff_norm < calib_config.blob_detect_centroid_norm_cutoff
                break
            end
        end  
        if calib_config.verbose > 2 && it == calib_config.blob_detect_centroid_it_cutoff
            warning('Blob detect centroid refinement iterations hit cutoff before converging!!!');
        end 
                               
        % Get iteratively refined second moment matrix.
        r1_ellipse = maxima_r;
        r2_ellipse = maxima_r;
        rot = 0;
        for it = 1:calib_config.blob_detect_M_it_cutoff          
            [eig_ratio,r1_ellipse,r2_ellipse,rot,M] = second_moment_params(sub_array_dx, ...
                                                                           sub_array_dy, ...
                                                                           x_sub_array, ...
                                                                           y_sub_array, ...
                                                                           maxima_x, ...
                                                                           maxima_y, ...
                                                                           maxima_r, ...
                                                                           r1_ellipse, ...
                                                                           r2_ellipse, ... 
                                                                           rot);   
            if isnan(eig_ratio) || eig_ratio > calib_config.blob_detect_eig_ratio_cutoff
                break
            end
        end              
              
        % Filter out edge response again
        if isnan(eig_ratio) || eig_ratio > calib_config.blob_detect_eig_ratio_cutoff
            continue
        end
        
        % Remove any out of range idx
        if maxima_r_idx < 1 || maxima_r_idx > num_scales || ...
           maxima_x < 1 || maxima_x > size(array,2) || ...
           maxima_y < 1 || maxima_y > size(array,1)
            continue;
        end
        
        % Before storing blob, make sure there isn't another blob near
        % this one. This is ad-hoc clustering but is typically "good enough"
        dist_d = sqrt(([blobs.x]-maxima_x).^2 + ...
                      ([blobs.y]-maxima_y).^2);
        dist_r = abs([blobs.r]-maxima_r);        
        if all(dist_d > calib_config.blob_detect_d_cluster | ...
               dist_r > calib_config.blob_detect_r_cluster)
            blobs(end+1).x = maxima_x; %#ok<AGROW>
            blobs(end).y = maxima_y;
            blobs(end).r = maxima_r;
            blobs(end).M = M;
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