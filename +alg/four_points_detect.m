function [four_points_p,four_points_debug] = four_points_detect(array,calib_config)
    % Obtains the locations of the four points (fiducial markers) around 
    % the calibration board.
    % 
    % Inputs:
    %   array - array; MxN array
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   four_points_p - array; 4x2 array of four points in pixel
    %       coordinates
    %   four_points_debug - struct; Used for debugging purposes
    %       .blobs - struct; output from alg.blob_detect().
    %       .ellipses - struct;
    %           .x - scalar; x location of ellipse
    %           .y - scalar; y location of ellipse
    %           .r1 - scalar; major axis in pixels
    %           .r2 - scalar; minor axis in pixels
    %           .rot - scalar; rotation of major axis in radians
    %       .patch_matches - cell; 4x2 cell of patches. First column is 
    %           detected patch, circularly shifted to match the template. 
    %           The second column is the template with radius shift 
    %           applied.
       
    % Read marker config and marker templates
    marker_config = util.read_data(calib_config.marker_config_path);
    marker_templates = util.read_data(calib_config.marker_templates_path);
    
    % Make sure there are four marker templates
    if ~isfield(marker_templates,'polar_patches') || length(marker_templates.polar_patches) ~= 4
        error('There must be four templates in the marker templates file!');
    end
          
    % Get normalized radius and theta samples used for sampling polar patch
    r_samples = linspace(marker_config.radius_norm_range(1), ...
                         marker_config.radius_norm_range(2), ...
                         marker_config.radius_num_samples);
    % For theta, sample 1 more and then remove it; this allows [0 2*pi)
    theta_samples = linspace(0,2*pi,marker_config.theta_num_samples+1);
    theta_samples(end) = []; 
    
    % Apply marker padding to r_samples. This allows some wiggle in case 
    % the size of blob is slightly off
    if 2*calib_config.marker_padding >= length(r_samples)
        error(['marker_padding size of: ' num2str(calib_config.marker_padding) ' ' ...
               'is too large. Please reduce the amount of padding.']);
    end    
    r_samples = r_samples(calib_config.marker_padding+1: ...
                          end-calib_config.marker_padding);
    
    % Get blobs which should detect center of fiducial marker
    blobs = alg.blob_detect(array,calib_config);
        
    % Get ellipses
    ellipses = struct('x',{},'y',{},'r1',{},'r2',{},'rot',{});
    % Also store "cost" of ellipse which should indicate which blobs are 
    % more ellipse-like
    ellipse_costs = [];
    for i = 1:length(blobs)
        % First, get sub array containing blob
        half_window = ceil(4*blobs(i).r);
        sub_array_l = round(blobs(i).x)-half_window;
        sub_array_r = round(blobs(i).x)+half_window;   
        sub_array_t = round(blobs(i).y)-half_window;
        sub_array_b = round(blobs(i).y)+half_window;     
        if sub_array_l < 1 || sub_array_r > size(array,2) || ...
           sub_array_t < 1 || sub_array_b > size(array,1)
            continue
        end    
        sub_array = array(sub_array_t:sub_array_b,sub_array_l:sub_array_r);        
        
        % Refine center position of blob
        x_refined = blobs(i).x;
        y_refined = blobs(i).y;
        % Coordinates of points in sub_array
        [y_sub_array,x_sub_array] = ndgrid(sub_array_t:sub_array_b, ...
                                           sub_array_l:sub_array_r);
        % Get "centroid array" which is negative of sub array, scaled
        % between 0 and 1
        centroid_sub_array = -sub_array;
        min_centroid_sub_array = min(centroid_sub_array(:));
        max_centroid_sub_array = max(centroid_sub_array(:));
        centroid_sub_array = (centroid_sub_array-min_centroid_sub_array)./(max_centroid_sub_array-min_centroid_sub_array);
                
        for j = 1:10
            % Store previous
            x_refined_prev = x_refined;
            y_refined_prev = y_refined;
        
            % Get gaussian kernel
            kernel_gauss_blob = mvnpdf([x_sub_array(:) y_sub_array(:)],[x_refined y_refined],[(blobs(i).r)^2 0; 0 (blobs(i).r)^2]);
            kernel_gauss_blob = reshape(kernel_gauss_blob,size(sub_array));

            % Get refined x and y position
            centroid_sub_array_weighted = centroid_sub_array .* kernel_gauss_blob;
            centroid_sub_array_weighted_sum = sum(centroid_sub_array_weighted(:));

            x_refined = sum(sum(centroid_sub_array_weighted.*x_sub_array))/centroid_sub_array_weighted_sum;
            y_refined = sum(sum(centroid_sub_array_weighted.*y_sub_array))/centroid_sub_array_weighted_sum;

            % sqrt((x_refined_prev-x_refined)^2 + (y_refined_prev-y_refined)^2)
        end
                
        % Get initial guess of ellipse by using second moment matrix
        sub_array_x = alg.array_grad(sub_array,'x');
        sub_array_y = alg.array_grad(sub_array,'y');
        kernel_gauss_blob = mvnpdf([x_sub_array(:) y_sub_array(:)],[x_refined y_refined],[(blobs(i).r)^2 0; 0 (blobs(i).r)^2]);
        kernel_gauss_blob = reshape(kernel_gauss_blob,size(sub_array));
        M(1,1) = sum(kernel_gauss_blob(:).*sub_array_x(:).^2);
        M(1,2) = sum(kernel_gauss_blob(:).*sub_array_x(:).*sub_array_y(:));
        M(2,2) = sum(kernel_gauss_blob(:).*sub_array_y(:).^2);
        M(2,1) = M(1,2);        
        if rank(M) < 2
            continue
        end
                
        % Get major/minor axis and rotation of ellipse
        [V,D] = eig(M);
        [D,idx_sorted] = sort(diag(D));
        V = V(:,idx_sorted);        
        % Ellipse sides are proportional to sqrt of eigenvalues
        ellise_scale_factor = 2*blobs(i).r/(sqrt(D(1))+sqrt(D(2))); % have minor and major axis sum to diameter of blob
        r1 = sqrt(D(2))*ellise_scale_factor; % major axis radius
        r2 = sqrt(D(1))*ellise_scale_factor; % minor axis radius        
        % Rotation of major axis
        rot = -atan2(V(1,2),V(2,2));     
        
        % Now, do nonlinear refinement using initial guess
               
        % Get "cost" array - has high values along edge of ellipse. Use
        % smoothed, upsampled sub array to compute gradients in order
        % to improve convergence.
        cost_scale_factor = 2;
        sub_array_up = imresize(alg.array_gauss(sub_array,r2/2),cost_scale_factor);
        cost_sub_array = sqrt(alg.array_grad(sub_array_up,'x').^2 + ...
                              alg.array_grad(sub_array_up,'y').^2);
        cost_sub_array_x = alg.array_grad(cost_sub_array,'x');
        cost_sub_array_y = alg.array_grad(cost_sub_array,'y');
        I_cost_sub_array = griddedInterpolant({1:size(cost_sub_array,1),1:size(cost_sub_array,2)}, ...
                                              cost_sub_array, 'linear','none');
        I_cost_sub_array_x = griddedInterpolant({1:size(cost_sub_array_x,1),1:size(cost_sub_array_x,2)}, ...
                                                cost_sub_array_x, 'linear','none');
        I_cost_sub_array_y = griddedInterpolant({1:size(cost_sub_array_y,1),1:size(cost_sub_array_y,2)}, ...
                                                cost_sub_array_y, 'linear','none');
                   
        % Perform gradient ascent with backtracking
        p = [x_refined y_refined r1 r2 rot]'; % initialize parameter vector with current ellipse
        for it = 1:calib_config.marker_it_cutoff
            % Set p_init to p from previous iteration
            p_init = p;

            % Get ellipse points at p_init
            x_init = cost_scale_factor*(p_init(3)*cos(p_init(5))*cos(theta_samples) - p_init(4)*sin(p_init(5))*sin(theta_samples) + (p_init(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
            y_init = cost_scale_factor*(p_init(3)*sin(p_init(5))*cos(theta_samples) + p_init(4)*cos(p_init(5))*sin(theta_samples) + (p_init(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));

            % Compute gradient at p_init
            dc_dx = I_cost_sub_array_x(y_init',x_init');
            dc_dy = I_cost_sub_array_y(y_init',x_init');
            dx_dp = [ones(length(theta_samples),1)  zeros(length(theta_samples),1) cos(p_init(5))*cos(theta_samples') -sin(p_init(5))*sin(theta_samples') -p_init(3)*sin(p_init(5))*cos(theta_samples')-p_init(4)*cos(p_init(5))*sin(theta_samples')];
            dy_dp = [zeros(length(theta_samples),1) ones(length(theta_samples),1)  sin(p_init(5))*cos(theta_samples')  cos(p_init(5))*sin(theta_samples')  p_init(3)*cos(p_init(5))*cos(theta_samples')-p_init(4)*sin(p_init(5))*sin(theta_samples')];
            grad = sum(dc_dx.*dx_dp + dc_dy.*dy_dp)';

            % Get initial cost; this is used in backtracking
            cost_init = sum(I_cost_sub_array(y_init,x_init));

            % Perform backtracking 
            n = 1;
            p = p_init + n*grad;
            x = cost_scale_factor*(p(3)*cos(p(5))*cos(theta_samples) - p(4)*sin(p(5))*sin(theta_samples) + (p(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
            y = cost_scale_factor*(p(3)*sin(p(5))*cos(theta_samples) + p(4)*cos(p(5))*sin(theta_samples) + (p(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));
            while all(x >= 1 & x <= size(cost_sub_array,2) & ...
                      y >= 1 & y <= size(cost_sub_array,1)) && ...
                  sum(I_cost_sub_array(y',x')) < cost_init+(1/2)*n*dot(grad,grad) % Backtracking guaranteed to exit this condition eventually
                % Half step size
                n = (1/2)*n;
                p = p_init + n*grad;
                x = cost_scale_factor*(p(3)*cos(p(5))*cos(theta_samples) - p(4)*sin(p(5))*sin(theta_samples) + (p(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
                y = cost_scale_factor*(p(3)*sin(p(5))*cos(theta_samples) + p(4)*cos(p(5))*sin(theta_samples) + (p(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));
            end
               
            % Exit if the ellipse went out of the bounding box
            if any(x < 1 | x > size(cost_sub_array,2) | ...
                   y < 1 | y > size(cost_sub_array,1))
                p(:) = NaN;
                break
            end
               
            %{
            % Get ellipse points at p_init
            figure(1);
            x_plot = cost_scale_factor*(p(3)*cos(p(5))*cos(theta_samples) - p(4)*sin(p(5))*sin(theta_samples) + (p(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
            y_plot = cost_scale_factor*(p(3)*sin(p(5))*cos(theta_samples) + p(4)*cos(p(5))*sin(theta_samples) + (p(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));
            subplot(1,2,2);
            imshow(cost_sub_array,[]);
            hold on;
            plot(x_plot,y_plot,'-r');
            plot(x,y,'-g');
            norm(p-p_init)
            drawnow 
            %}

            % Exit if change in distance is small
            diff_norm = norm(p-p_init);            
            if calib_config.verbose > 2
                disp(['Marker detect iteration #: ' num2str(it)]);
                disp(['Difference norm for nonlinear parameter refinement: ' num2str(diff_norm)]);
            end
            if diff_norm < calib_config.marker_norm_cutoff
                break
            end
        end
        if calib_config.verbose > 2 && it == calib_config.marker_it_cutoff
            warning('Marker iterations hit cutoff before converging!!!');
        end
                
        % Check if any params are NaNs or if minor/major axes is very small
        if any(isnan(p)) || ...
           p(3) <= calib_config.ellipse_detect_r1_cutoff || ...
           p(4) <= calib_config.ellipse_detect_r2_cutoff
            continue
        end
        
        
        % Before storing ellipse, make sure there isn't another ellipse near
        % this one. This is ad-hoc clustering and typically "good enough"
        dist_d = sqrt(([ellipses.x]-p(1)).^2 + ...
                      ([ellipses.y]-p(2)).^2);
        dist_r1 = abs([ellipses.r1]-p(3));        
        dist_r2 = abs([ellipses.r2]-p(4));        
        dist_rot = abs([ellipses.rot]-p(5));        
        if all(dist_d > 1 | ...
               dist_r1 > 1 | ...
               dist_r2 > 1 | ...
               dist_rot > 2*pi/36)        
            % Compute "cost" of each ellipse

            % Get gaussian kernel in the shape of the ellipse
            rotation = [cos(p(5)) -sin(p(5));  ...
                        sin(p(5))  cos(p(5))];
            variance = [p(3)^2 0; ...
                        0      p(4)^2];
            covariance = rotation*variance*rotation';              
            if rank(covariance) < 2
                continue
            end        
            [y_window_up,x_window_up] = ndgrid(1:size(sub_array_up,1),1:size(sub_array_up,2));
            x_window_up = (x_window_up./cost_scale_factor + 1/2*(1-1/cost_scale_factor)) -1 + sub_array_l;
            y_window_up = (y_window_up./cost_scale_factor + 1/2*(1-1/cost_scale_factor)) -1 + sub_array_t;

            kernel_gauss_ellipse = mvnpdf([x_window_up(:) y_window_up(:)],[p(1) p(2)],covariance);
            kernel_gauss_ellipse = reshape(kernel_gauss_ellipse,size(sub_array_up));

            % Compute and store ellipse cost       
            cost_sub_array_ellipse = cost_sub_array .* kernel_gauss_ellipse;          
            cost_sub_array_ellipse = (cost_sub_array_ellipse - min(cost_sub_array_ellipse(:)))./(max(cost_sub_array_ellipse(:)) - min(cost_sub_array_ellipse(:)));
            x = cost_scale_factor*(p(3)*cos(p(5))*cos(theta_samples) - p(4)*sin(p(5))*sin(theta_samples) + (p(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
            y = cost_scale_factor*(p(3)*sin(p(5))*cos(theta_samples) + p(4)*cos(p(5))*sin(theta_samples) + (p(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));
            ellipse_costs(end+1) = min(alg.array_interp(cost_sub_array_ellipse,[x' y'],'linear')); %#ok<AGROW>

            % Store refined outputs
            ellipses(end+1).x = p(1); %#ok<AGROW>
            ellipses(end).y = p(2);
            ellipses(end).r1 = p(3);
            ellipses(end).r2 = p(4);
            ellipses(end).rot = p(5);
        end
    end 
    
    % Threshold number of ellipses
    [~,idx_ellipse] = sort(ellipse_costs,'descend');
    ellipses = ellipses(idx_ellipse(1:min(calib_config.ellipse_detect_num_cutoff,end)));
    
    % Get polar patches based on ellipses
    polar_patches = cell(1,length(ellipses));
    for i = 1:length(ellipses)          
        % Make xform to apply to coordinates of circle: rotation * scaling
        rotation = [cos(ellipses(i).rot) -sin(ellipses(i).rot);  ...
                    sin(ellipses(i).rot)  cos(ellipses(i).rot)];
        scaling = [ellipses(i).r1 0; ...
                   0 ellipses(i).r2];
        xform = rotation * scaling;

        % Get normalized coordinates
        x = bsxfun(@times,cos(theta_samples)',r_samples); 
        y = bsxfun(@times,sin(theta_samples)',r_samples); 

        % Apply transformation and translate to center of ellipse
        p_affine = xform * vertcat(x(:)',y(:)');
        p_affine(1,:) = p_affine(1,:)+ellipses(i).x;
        p_affine(2,:) = p_affine(2,:)+ellipses(i).y;

        % Check for out of bounds points
        if any(p_affine(1,:) < 1 | p_affine(1,:) > size(array,2) | ...
               p_affine(2,:) < 1 | p_affine(2,:) > size(array,1))
            continue
        end
            
        % Resample
        polar_patch = alg.array_interp(array, p_affine', 'linear');
        polar_patches{i} = reshape(polar_patch,marker_config.theta_num_samples,[]);    
    end
    
    % Cross correlate each polar patch with 4 templates. 
    cc_mat = -Inf(length(polar_patches),4);
    i_idx_mat = -1*ones(length(polar_patches),4);  
    j_idx_mat = -1*ones(length(polar_patches),4);  
    for i = 1:length(polar_patches)
        % Make sure polar patch isn't empty, which can happen if sampling
        % points were outside of field of view.
        if isempty(polar_patches{i})
            continue
        end
        
        % Normalize polar patch 
        polar_patches{i} = polar_patches{i} - mean(polar_patches{i}(:));
        polar_patches{i} = polar_patches{i}./norm(polar_patches{i}(:));
        
        % Compare to four templates with circular cross correlation over
        % theta and slide over radius if padding is provided.
        for j = 1:4
            cc_buf = zeros(marker_config.theta_num_samples,2*calib_config.marker_padding+1);
            for m = 1:2*calib_config.marker_padding+1
                for n = 1:size(polar_patches{i},2)
                    cc_buf(:,m) = cc_buf(:,m) + ifft(fft(polar_patches{i}(:,n)).*conj(fft(marker_templates.polar_patches{j}(:,m+n-1))));
                end
            end
            
            % Get max correlation and store in cc_mat
            [i_max, j_max] = find(cc_buf == max(cc_buf(:)),1);
            cc_mat(i,j) = cc_buf(i_max,j_max);
            i_idx_mat(i,j) = i_max; % Theta shift
            j_idx_mat(i,j) = j_max; % Radial shift
        end
    end   
        
    % Get best matches
    patch_matches = cell(4,2);
    four_points_p = zeros(4,2);
    for i = 1:4
        % TODO: possibly use max difference between best and 2nd best match
        % per template instead of just the best match overall
        
        % Get max value
        [i_max, j_max] = find(cc_mat == max(cc_mat(:)),1);
                     
        % Set coordinates
        four_points_p(j_max,:) = [ellipses(i_max).x ellipses(i_max).y];
                                       
        % "disable" this patch and template
        cc_mat(i_max,:) = -Inf; % patch
        cc_mat(:,j_max) = -Inf; % template
        
        % Store best patch matches
        patch_matches{j_max,1} = circshift(polar_patches{i_max},-(i_idx_mat(i_max,j_max)-1));
        patch_matches{j_max,2} = marker_templates.polar_patches{j_max}(:,j_idx_mat(i_max,j_max):j_idx_mat(i_max,j_max)+length(r_samples)-1);
    end     
    
    % Set debugging output
    four_points_debug.blobs = blobs;
    four_points_debug.ellipses = ellipses;
    four_points_debug.patch_matches = patch_matches;
end