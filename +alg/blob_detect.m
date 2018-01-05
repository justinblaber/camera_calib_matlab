function blobs = blob_detect(array,calib_config)
    % Performs blob detection. By default this returns dark blobs. If you
    % want both dark and light blobs you can process the array and the 
    % negative of the array and then merge the results.
    % 
    % Inputs:
    %   array - array; MxN array
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   blobs - struct; contains:
    %       .x - scalar; x location of blob
    %       .y - scalar; y location of blob
    %       .r - scalar; radius of blob in pixels
    
    % TODO: for the defaults, you can, instead of setting them here, set 
    % them in a wrapper function to make this function more general...
    
    % Set num_cutoff
    if calib_config.blob_detect_num_cutoff == realmax % This is the flag to set default
        % 4 markers + ~number of squares + 500 blob cushion
        num_cutoff = 4 + calib_config.num_squares_height*calib_config.num_squares_width + 500;            
    else
        num_cutoff = calib_config.blob_detect_num_cutoff;
    end
    
    % Set radius range
    r1 = calib_config.blob_detect_r1;    
    if calib_config.blob_detect_r2 == realmax % This is the flag to set default
        % Markers should be relatively small
        r2 = min(size(array))/40 + r1;
    else
        r2 = calib_config.blob_detect_r2;
    end  
    
    % Make sure r2 is an integer number of steps away from r1
    num_scales = ceil((r2-r1)/calib_config.blob_detect_step+1);
    r2 = r1 + (num_scales-1)*calib_config.blob_detect_step;
            
    % Create scale normalized LoG stack
    stack_LoG = zeros([size(array) num_scales]);
    for i = 1:num_scales
        sigma = (calib_config.blob_detect_step*(i-1)+r1)/sqrt(2);
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
    
    % Get most powerful blob responses specified by num_cutoff
    maxima_vals = stack_LoG(maxima_idx);
    [~,maxima_sorted_idx] = sort(maxima_vals,'descend');
    maxima_sorted_idx = maxima_sorted_idx(1:min(num_cutoff,end));
    maxima_sorted_idx = maxima_sorted_idx(maxima_vals(maxima_sorted_idx) > 0); % Make sure values are positive
    maxima_x_init = maxima_x_init(maxima_sorted_idx);
    maxima_y_init = maxima_y_init(maxima_sorted_idx);
    maxima_r_idx_init = maxima_r_idx_init(maxima_sorted_idx);
    
    % Perform sub-pixel refinement    
    maxima_x = maxima_x_init;
    maxima_y = maxima_y_init;
    maxima_r_idx = maxima_r_idx_init;
    % Initialize "box" to interpolate in order to compute finite difference
    % approximations to gradient/hessian and also get interpolator
    [y_box,x_box,r_idx_box] = ndgrid(-1:1:1,-1:1:1,-1:1:1);
    I_stack_LoG = griddedInterpolant({1:size(stack_LoG,1),1:size(stack_LoG,2),1:size(stack_LoG,3)}, ...
                                      stack_LoG,'cubic','none');
    for i = 1:length(maxima_x)
        % Perform gauss newton iterations until convergence; during 
        % iterations, set invalid points to NaNs
        for it = 1:calib_config.blob_detect_it_cutoff
            % Get finite difference coordinates for interpolation
            x_fd = x_box + maxima_x(i);
            y_fd = y_box + maxima_y(i);
            r_idx_fd = r_idx_box + maxima_r_idx(i);
            
            % Check to make sure finite difference box is in range
            if any(x_fd(:) < 1 | x_fd(:) > size(stack_LoG,2) | ...
                   y_fd(:) < 1 | y_fd(:) > size(stack_LoG,1) | ...
                   r_idx_fd(:) < 1 | r_idx_fd(:) > size(stack_LoG,3))
                maxima_x(i) = NaN;
                maxima_y(i) = NaN;
                maxima_r_idx(i) = NaN;
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
                maxima_x(i) = NaN;
                maxima_y(i) = NaN;
                maxima_r_idx(i) = NaN;
                break
            end
            
            % Get optimized locations
            maxima_x(i) = maxima_x(i)+delta_p(1);
            maxima_y(i) = maxima_y(i)+delta_p(2);
            maxima_r_idx(i) = maxima_r_idx(i)+delta_p(3);
            
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
    end
        
    % Remove any NaNs and out of range idx
    idx_bad = isnan(maxima_x) | isnan(maxima_y) | isnan(maxima_r_idx) | ...
              maxima_r_idx < 1 | maxima_r_idx > num_scales | ...
              maxima_x < 1 | maxima_x > size(array,2) | ...
              maxima_y < 1 | maxima_y > size(array,1);
    maxima_x(idx_bad) = [];
    maxima_y(idx_bad) = [];
    maxima_r_idx(idx_bad) = [];
    
    % Store outputs
    blobs = struct('x',{},'y',{},'r',{});
    for i = 1:length(maxima_x)
        % Before storing blob, make sure there isn't another blob near
        % this one. This is ad-hoc clustering and typically "good enough"
        dist_d = sqrt(([blobs.x]-maxima_x(i)).^2 + ...
                      ([blobs.y]-maxima_y(i)).^2);
        dist_r = abs([blobs.r]-((r2-r1)/(num_scales-1)*(maxima_r_idx(i)-1)+r1));        
        if all(dist_d > calib_config.blob_detect_d_cluster | ...
               dist_r > calib_config.blob_detect_r_cluster)
            blobs(end+1).x = maxima_x(i); %#ok<AGROW>
            blobs(end).y = maxima_y(i);
            blobs(end).r = (r2-r1)/(num_scales-1)*(maxima_r_idx(i)-1)+r1;
        end
    end
end