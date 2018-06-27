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
    %       .patch_matches - struct;
    %           .patch - array; detected patch, circularly shifted to match
    %               the template.
    %           .template - array; template with radius shift applied.
    %           .ellipse - struct; ellipse corresponding to detected patch
    %           .cc_val - scalar; cross correlation coefficient
       
    % Read marker config and marker templates
    marker_config = util.read_data(calib_config.marker_config_path);
    marker_templates = util.read_data(calib_config.marker_templates_path);
    
    % Make sure there are four marker templates
    if ~isfield(marker_templates,'polar_patches') || length(marker_templates.polar_patches) ~= 4
        error('There must be four templates in the marker templates file!');
    end
    
    % Rescale array
    if calib_config.four_point_detect_scaled_array_min_size == realmax
        scale_factor = 1;
    else
        scale_factor = calib_config.four_point_detect_scaled_array_min_size/min(size(array));
        array = imresize(array,scale_factor);
    end
         
    % Get blobs which should detect center of fiducial markers
    blobs = alg.blob_detect(array,calib_config);
        
    % Get ellipses
    ellipses = struct('x',{},'y',{},'r1',{},'r2',{},'rot',{});    
    % For theta, sample 1 more and then remove it; this allows [0 2*pi)
    ellipse_theta_samples = linspace(0,2*pi,calib_config.ellipse_detect_theta_num_samples+1);
    ellipse_theta_samples(end) = [];     
    % Also store "cost" of ellipse which should indicate which blobs are 
    % more ellipse-like
    ellipse_costs = [];
    for i = 1:length(blobs)    
        % Get initial guess of ellipse by using second moment matrix
        % Get major/minor axis and rotation of ellipse
        [V,D] = eig(blobs(i).M);
        [D,idx_sorted] = sort(diag(D));
        V = V(:,idx_sorted);        
        % Ellipse sides are proportional to sqrt of eigenvalues
        ellipse_scale_factor = 2*blobs(i).r/(sqrt(D(1))+sqrt(D(2))); % have minor and major axis sum to diameter of blob
        r1 = sqrt(D(2))*ellipse_scale_factor; % major axis radius
        r2 = sqrt(D(1))*ellipse_scale_factor; % minor axis radius        
        % Rotation of major axis
        rot = -atan2(V(1,2),V(2,2));     
        
        % Now, do nonlinear refinement using initial guess                     
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
        
        % Normalize sub array to ensure gradients are consistent
        sub_array = (sub_array-min(sub_array(:)))/(max(sub_array(:))-min(sub_array(:)));   
               
        % Get "cost" array - has high values along edge of ellipse. Use
        % smoothed, upsampled sub array to compute gradients in order
        % to improve convergence.
        cost_scale_factor = 2; % MUST BE INTEGER
        sub_array_up = imresize(alg.array_gauss(sub_array,r2/2),cost_scale_factor);
        cost_sub_array = sqrt(alg.array_grad(sub_array_up,'x').^2 + ...
                              alg.array_grad(sub_array_up,'y').^2);
        cost_sub_array_dx = alg.array_grad(cost_sub_array,'x');
        cost_sub_array_dy = alg.array_grad(cost_sub_array,'y');
        % Get interpolants for speed
        I_cost_sub_array = griddedInterpolant({1:size(cost_sub_array,1),1:size(cost_sub_array,2)}, ...
                                              cost_sub_array,'cubic','none');
        I_cost_sub_array_dx = griddedInterpolant({1:size(cost_sub_array_dx,1),1:size(cost_sub_array_dx,2)}, ...
                                                 cost_sub_array_dx,'cubic','none');
        I_cost_sub_array_dy = griddedInterpolant({1:size(cost_sub_array_dy,1),1:size(cost_sub_array_dy,2)}, ...
                                                 cost_sub_array_dy,'cubic','none');                   
        % Perform gradient ascent with backtracking
        p = [blobs(i).x blobs(i).y r1 r2 rot]'; % initialize parameter vector with current ellipse
        for it = 1:calib_config.ellipse_detect_it_cutoff
            % Store previous p
            p_prev = p;

            % Get ellipse points at p_prev
            x_prev = cost_scale_factor*(p_prev(3)*cos(p_prev(5))*cos(ellipse_theta_samples) - p_prev(4)*sin(p_prev(5))*sin(ellipse_theta_samples) + (p_prev(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
            y_prev = cost_scale_factor*(p_prev(3)*sin(p_prev(5))*cos(ellipse_theta_samples) + p_prev(4)*cos(p_prev(5))*sin(ellipse_theta_samples) + (p_prev(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));

            % Compute gradient at p_prev
            dc_dx = I_cost_sub_array_dx(y_prev',x_prev');
            dc_dy = I_cost_sub_array_dy(y_prev',x_prev');
            dx_dp = [ones(length(ellipse_theta_samples),1)  zeros(length(ellipse_theta_samples),1) cos(p_prev(5))*cos(ellipse_theta_samples') -sin(p_prev(5))*sin(ellipse_theta_samples') -p_prev(3)*sin(p_prev(5))*cos(ellipse_theta_samples')-p_prev(4)*cos(p_prev(5))*sin(ellipse_theta_samples')];
            dy_dp = [zeros(length(ellipse_theta_samples),1) ones(length(ellipse_theta_samples),1)  sin(p_prev(5))*cos(ellipse_theta_samples')  cos(p_prev(5))*sin(ellipse_theta_samples')  p_prev(3)*cos(p_prev(5))*cos(ellipse_theta_samples')-p_prev(4)*sin(p_prev(5))*sin(ellipse_theta_samples')];
            grad = sum(dc_dx.*dx_dp + dc_dy.*dy_dp)';

            % Perform backtracking 
            cost_init = sum(I_cost_sub_array(y_prev,x_prev));
            n = 1;
            p = p_prev + n*grad;
            x = cost_scale_factor*(p(3)*cos(p(5))*cos(ellipse_theta_samples) - p(4)*sin(p(5))*sin(ellipse_theta_samples) + (p(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
            y = cost_scale_factor*(p(3)*sin(p(5))*cos(ellipse_theta_samples) + p(4)*cos(p(5))*sin(ellipse_theta_samples) + (p(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));
            while all(x >= 1 & x <= size(cost_sub_array,2) & ...
                      y >= 1 & y <= size(cost_sub_array,1)) && ...
                  sum(I_cost_sub_array(y',x')) < cost_init+(1/2)*n*dot(grad,grad) % Backtracking guaranteed to exit this condition eventually
                % Half step size
                n = (1/2)*n;
                p = p_prev + n*grad;
                x = cost_scale_factor*(p(3)*cos(p(5))*cos(ellipse_theta_samples) - p(4)*sin(p(5))*sin(ellipse_theta_samples) + (p(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
                y = cost_scale_factor*(p(3)*sin(p(5))*cos(ellipse_theta_samples) + p(4)*cos(p(5))*sin(ellipse_theta_samples) + (p(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));
            end
               
            % Exit if the ellipse went out of the bounding box
            if any(x < 1 | x > size(cost_sub_array,2) | ...
                   y < 1 | y > size(cost_sub_array,1))
                p(:) = NaN;
                break
            end
               
            % Exit if change in distance is small
            diff_norm = norm(p-p_prev);            
            if calib_config.verbose > 2
                disp(['Marker detect iteration #: ' num2str(it)]);
                disp(['Difference norm for nonlinear parameter refinement: ' num2str(diff_norm)]);
            end
            if diff_norm < calib_config.ellipse_detect_norm_cutoff
                break
            end
        end
        if calib_config.verbose > 2 && it == calib_config.ellipse_detect_it_cutoff
            warning('Marker iterations hit cutoff before converging!!!');
        end
                
        % Check if any params are NaNs or if minor/major axes is very small
        if any(isnan(p)) || ...
           p(3) <= calib_config.ellipse_detect_r1_cutoff || ...
           p(4) <= calib_config.ellipse_detect_r2_cutoff
            continue
        end
                
        % Before storing ellipse, make sure there isn't another ellipse 
        % near this one. This is ad-hoc clustering and is typically 
        % "good enough"
        dist_d = sqrt(([ellipses.x]-p(1)).^2 + ...
                      ([ellipses.y]-p(2)).^2);
        dist_r1 = abs([ellipses.r1]-p(3));        
        dist_r2 = abs([ellipses.r2]-p(4));        
        dist_rot = abs([ellipses.rot]-p(5));        
        if all(dist_d > calib_config.ellipse_detect_d_cluster | ...
               dist_r1 > calib_config.ellipse_detect_r1_cluster | ...
               dist_r2 > calib_config.ellipse_detect_r2_cluster | ...
               dist_rot > calib_config.ellipse_detect_rot_cluster)        
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
            x_window_up = (x_window_up./cost_scale_factor + 1/2*(1-1/cost_scale_factor)) - 1 + sub_array_l;
            y_window_up = (y_window_up./cost_scale_factor + 1/2*(1-1/cost_scale_factor)) - 1 + sub_array_t;
            kernel_gauss = mvnpdf([x_window_up(:) y_window_up(:)], ...
                                  [p(1) p(2)], ...
                                  covariance);
            kernel_gauss = reshape(kernel_gauss,size(sub_array_up));

            % TODO: think of better ellipse cost
            
            % Compute and store ellipse cost. Taking the minimum value
            % seems to be a reasonable choice, as ellipses should not have 
            % "dips" unlike other features.
            cost_sub_array_ellipse = cost_sub_array.*kernel_gauss;          
            cost_sub_array_ellipse = (cost_sub_array_ellipse - min(cost_sub_array_ellipse(:)))/(max(cost_sub_array_ellipse(:)) - min(cost_sub_array_ellipse(:)));
            x = cost_scale_factor*(p(3)*cos(p(5))*cos(ellipse_theta_samples) - p(4)*sin(p(5))*sin(ellipse_theta_samples) + (p(1)-sub_array_l+1) - 1/2*(1-1/cost_scale_factor));
            y = cost_scale_factor*(p(3)*sin(p(5))*cos(ellipse_theta_samples) + p(4)*cos(p(5))*sin(ellipse_theta_samples) + (p(2)-sub_array_t+1) - 1/2*(1-1/cost_scale_factor));
            ellipse_costs(end+1) = min(alg.array_interp(cost_sub_array_ellipse,[x' y'],'linear')); %#ok<AGROW>

            % Store ellipse
            ellipses(end+1).x = p(1); %#ok<AGROW>
            ellipses(end).y = p(2);
            ellipses(end).r1 = p(3);
            ellipses(end).r2 = p(4);
            ellipses(end).rot = p(5);
        end
    end 
    
    % Threshold number of ellipses based on their cost
    [~,idx_ellipse] = sort(ellipse_costs,'descend');
    ellipses = ellipses(idx_ellipse(1:min(calib_config.ellipse_detect_num_cutoff,end)));
    
    if length(ellipses) < 4
        error(['At least four ellipses must be detected; only: ' ...
               num2str(length(ellipses)) ' were found.']);
    end
    
    % Get polar patches based on ellipses
    polar_patches = cell(1,length(ellipses));    
    % Get normalized radius and theta samples used for sampling polar patch
    polarpatch_r_samples = linspace(marker_config.radius_norm_range(1), ...
                                    marker_config.radius_norm_range(2), ...
                                    marker_config.radius_num_samples);
    % For theta, sample 1 more and then remove it; this allows [0 2*pi)
    polarpatch_theta_samples = linspace(0,2*pi,marker_config.theta_num_samples+1);
    polarpatch_theta_samples(end) = [];     
    % Apply marker padding to r_samples. This allows some wiggle in case 
    % the size of blob is slightly off
    if 2*calib_config.marker_padding >= length(polarpatch_r_samples)
        error(['marker_padding size of: ' num2str(calib_config.marker_padding) ' ' ...
               'is too large. Please reduce the amount of padding.']);
    end    
    polarpatch_r_samples = polarpatch_r_samples(calib_config.marker_padding+1: ...
                                                end-calib_config.marker_padding);    
    for i = 1:length(ellipses)          
        % Make xform to apply to coordinates of circle: rotation * scaling
        rotation = [cos(ellipses(i).rot) -sin(ellipses(i).rot);  ...
                    sin(ellipses(i).rot)  cos(ellipses(i).rot)];
        scaling = [ellipses(i).r1 0; ...
                   0              ellipses(i).r2];
        xform = rotation * scaling;

        % Get normalized coordinates
        x = bsxfun(@times,cos(polarpatch_theta_samples)',polarpatch_r_samples); 
        y = bsxfun(@times,sin(polarpatch_theta_samples)',polarpatch_r_samples); 

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
        
    % Get best matches and store four points
    patch_matches = struct('patch',cell(4,1), ...
                           'template',cell(4,1), ...
                           'ellipse',cell(4,1), ...
                           'cc_val',cell(4,1));
    four_points_p = zeros(4,2);
    for i = 1:4
        % TODO: possibly use max difference between best and 2nd best match
        % per template instead of just the best match overall
        
        % Get max value
        [i_max, j_max] = find(cc_mat == max(cc_mat(:)),1);
                     
        % Set coordinates
        four_points_p(j_max,:) = [ellipses(i_max).x ellipses(i_max).y];
                       
        % Store best patch matches
        patch_matches(j_max).patch = circshift(polar_patches{i_max},-(i_idx_mat(i_max,j_max)-1));
        patch_matches(j_max).template = marker_templates.polar_patches{j_max}(:,j_idx_mat(i_max,j_max):j_idx_mat(i_max,j_max)+length(polarpatch_r_samples)-1);
        patch_matches(j_max).ellipse = ellipses(i_max);
        patch_matches(j_max).cc_val = cc_mat(i_max,j_max);
                
        % "disable" this patch and template
        cc_mat(i_max,:) = -Inf; % patch
        cc_mat(:,j_max) = -Inf; % template
    end     
    
    % Set debugging output
    four_points_debug.blobs = blobs;
    four_points_debug.ellipses = ellipses;
    four_points_debug.patch_matches = patch_matches;
    
    % Rescale four_points based on scale_factor; debug stuff does not need to be rescaled
    four_points_p = four_points_p/scale_factor + 1/2*(1-1/scale_factor);
end