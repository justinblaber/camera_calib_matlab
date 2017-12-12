function [four_points_p,blobs,ellipses,patch_matches] = four_point_detect(array,calib_config)
    % Obtains the locations of the four points (fiducial markers) around 
    % the calibration board automatically.
    % 
    % Inputs:
    %   array - array; MxN array
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   four_points_p - array; 4x2 array of four points in pixel
    %       coordinates
    %   blobs - struct; output from alg.blob_detect()
    %   ellipses - struct; 
    %       .x - x location of ellipse
    %       .y - y location of ellipse
    %       .r1 - major axis in pixels
    %       .r2 - minor axis in pixels
    %       .rot - rotation of major axis in radians
    %   patch_matches - cell; 4x2 cell of patches. First column is detected
    %       patch, circularly shifted to match the template. The second
    %       column is the template with radius shift applied.
            
    % Read marker config and marker templates
    marker_config = util.read_data(calib_config.marker_config_path);
    marker_templates = util.read_data(calib_config.marker_templates_path);
    
    % Make sure there are four marker templates
    if ~isfield(marker_templates,'polar_patches') || length(marker_templates.polar_patches) ~= 4
        error('There must be four templates in the marker templates file!');
    end
                   
    % Normalize template patches - note that due to padding, the template
    % polar patches won't be normalized exactly correctly, but it shouldn't
    % be too big of a deal.
    for i = 1:4
        marker_templates.polar_patches{i} = marker_templates.polar_patches{i} - mean(marker_templates.polar_patches{i}(:));
        marker_templates.polar_patches{i} = marker_templates.polar_patches{i}./norm(marker_templates.polar_patches{i}(:));
    end
    
    % Get normalized radius and theta samples
    r_samples = linspace(marker_config.radius_norm_range(1), ...
                         marker_config.radius_norm_range(2), ...
                         marker_config.radius_num_samples);
    % For theta, sample 1 more and then remove it; this allows [0 2*pi)
    theta_samples = linspace(0,2*pi,marker_config.theta_num_samples+1);
    theta_samples(end) = []; 
    
    % Apply marker padding to r_samples. This allows some wiggle in case 
    % the size of blob is slightly off
    if 2*calib_config.marker_padding+1 >= length(r_samples)
        error(['marker_padding size of: ' num2str(calib_config.marker_padding) ' ' ...
               'is too large. Please reduce the amount of padding.']);
    end    
    r_samples = r_samples(calib_config.marker_padding+1: ...
                          end-calib_config.marker_padding);
    
    % Normalize array so gradients aren't ginormous
    array = (array-min(array(:)))./(max(array(:))-min(array(:)));
       
    % Get blobs which should detect center of fiducial marker
    blobs = alg.blob_detect(array,calib_config);
    
    % Get ellipses
    ellipses = struct('x',cell(1,length(blobs)), ...
                      'y',cell(1,length(blobs)), ...
                      'r1',cell(1,length(blobs)), ...
                      'r2',cell(1,length(blobs)), ...
                      'rot',cell(1,length(blobs)));
    array_grad_x = alg.array_grad(array,'x');
    array_grad_y = alg.array_grad(array,'y');
    for i = 1:length(blobs)
        % Get initial guess of ellipse by using second moment matrix
        window = 2*ceil(2*blobs(i).r)+1; % Dimensions must be odd 
        kernel_gauss = fspecial('gaussian',[window window],blobs(i).r);

        % Coordinates
        [y_window,x_window] = ndgrid(blobs(i).y-(window-1)/2:blobs(i).y+(window-1)/2, ...
                                     blobs(i).x-(window-1)/2:blobs(i).x+(window-1)/2);

        % Interpolate gradients
        blob_grad_x = reshape(alg.array_interp(array_grad_x,[x_window(:) y_window(:)],'bicubic'),window,window);
        blob_grad_y = reshape(alg.array_interp(array_grad_y,[x_window(:) y_window(:)],'bicubic'),window,window);
          
        % Compute second moment matrix
        M(1,1) = sum(kernel_gauss(:).*blob_grad_x(:).^2);
        M(1,2) = sum(kernel_gauss(:).*blob_grad_x(:).*blob_grad_y(:));
        M(2,2) = sum(kernel_gauss(:).*blob_grad_y(:).^2);
        M(2,1) = M(1,2);
                
        % Get eigenvalue decomposition of M
        [V,D] = eig(M);
        [D,idx_sorted] = sort(diag(D));
        V = V(:, idx_sorted);
        
        % Ellipse sides are proportional to sqrt of eigenvalues
        scale_factor = 2*blobs(i).r/(sqrt(D(1))+sqrt(D(2))); % have minor and major axis sum to diameter of blob
        r1 = sqrt(D(2))*scale_factor; % major axis radius
        r2 = sqrt(D(1))*scale_factor; % minor axis radius
        
        % Rotation of major axis
        rot = -atan2(V(1,2),V(2,2));     
                               
        % Now, do nonlinear refinement using initial guess
        % First, get sub array around blob
        width_ellipse = sqrt(r1^2*cos(rot)^2 + r2^2*sin(rot)^2);
        height_ellipse = sqrt(r1^2*sin(rot)^2 + r2^2*cos(rot)^2); 
        % bounding box
        scale_factor = 4;
        bb_l = floor(blobs(i).x - scale_factor*width_ellipse);
        bb_r = ceil(blobs(i).x + scale_factor*width_ellipse);
        bb_t = floor(blobs(i).y - scale_factor*height_ellipse);
        bb_b = ceil(blobs(i).y + scale_factor*height_ellipse);        
        if (bb_l < 1 || bb_r > size(array,2) || ...
            bb_t < 1 || bb_b > size(array,1))
            continue
        end        
        sub_array = array(bb_t:bb_b,bb_l:bb_r);
                
        % Get "cost" array
        cost_sub_array = alg.array_grad(alg.array_gauss(sub_array,r2/2),'x').^2 + ...
                         alg.array_grad(alg.array_gauss(sub_array,r2/2),'y').^2;
        
        % precompute gradients
        cost_sub_array_grad_x = alg.array_grad(cost_sub_array,'x');
        cost_sub_array_grad_y = alg.array_grad(cost_sub_array,'y');
                
        % Perform gradient ascent with backtracking
        % initialize parameter vector with current ellipse
        p = [blobs(i).x blobs(i).y r1 r2 rot]';
        for it = 1:calib_config.marker_it_cutoff
            % Set p_init to p from previous iteration
            p_init = p;
            
            % Get ellipse points at p_init
            x_init = p_init(3)*cos(p_init(5))*cos(theta_samples) - p_init(4)*sin(p_init(5))*sin(theta_samples) + (p_init(1)-bb_l+1);
            y_init = p_init(3)*sin(p_init(5))*cos(theta_samples) + p_init(4)*cos(p_init(5))*sin(theta_samples) + (p_init(2)-bb_t+1);
            
            % Compute gradient at p_init
            dc_dx = alg.array_interp(cost_sub_array_grad_x,[x_init' y_init'],'bicubic');
            dc_dy = alg.array_interp(cost_sub_array_grad_y,[x_init' y_init'],'bicubic');
            dx_dp = [ones(length(theta_samples),1)  zeros(length(theta_samples),1) cos(p_init(5))*cos(theta_samples') -sin(p_init(5))*sin(theta_samples') -p_init(3)*sin(p_init(5))*cos(theta_samples')-p_init(4)*cos(p_init(5))*sin(theta_samples')];
            dy_dp = [zeros(length(theta_samples),1) ones(length(theta_samples),1)  sin(p_init(5))*cos(theta_samples')  cos(p_init(5))*sin(theta_samples')  p_init(3)*cos(p_init(5))*cos(theta_samples')-p_init(4)*sin(p_init(5))*sin(theta_samples')];
            grad = sum(dc_dx.*dx_dp + dc_dy.*dy_dp)';
            
            % Get initial cost; this is used in backtracking
            cost_init = sum(alg.array_interp(cost_sub_array,[x_init' y_init'],'bicubic'));
                        
            % Perform backtracking 
            n = 1;
            p = p_init + n*grad;
            x = p(3)*cos(p(5))*cos(theta_samples) - p(4)*sin(p(5))*sin(theta_samples) + (p(1)-bb_l+1);
            y = p(3)*sin(p(5))*cos(theta_samples) + p(4)*cos(p(5))*sin(theta_samples) + (p(2)-bb_t+1);
            while all(x >= 1 & x <= size(cost_sub_array,2) & ...
                      y >= 1 & y <= size(cost_sub_array,1)) && ...
                  sum(alg.array_interp(cost_sub_array,[x' y'],'bicubic')) < cost_init+(1/2)*n*dot(grad,grad) % Backtracking guaranteed to exit this condition eventually
                % Half step size
                n = (1/2)*n;
                p = p_init + n*grad;
                x = p(3)*cos(p(5))*cos(theta_samples) - p(4)*sin(p(5))*sin(theta_samples) + (p(1)-bb_l+1);
                y = p(3)*sin(p(5))*cos(theta_samples) + p(4)*cos(p(5))*sin(theta_samples) + (p(2)-bb_t+1);
            end
               
            % Exit if the ellipse went out of the bounding box
            if any(x < 1 | x > size(cost_sub_array,2) | ...
                   y < 1 | y > size(cost_sub_array,1))
                p(:) = NaN;
                break
            end
            
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
                
        % Check if any params are NaNs which means sampling went out of
        % bounds
        if any(isnan(p))
            continue
        end

        % Store refined outputs
        ellipses(i).x = p(1);
        ellipses(i).y = p(2);
        ellipses(i).r1 = p(3);
        ellipses(i).r2 = p(4);
        ellipses(i).rot = p(5);
    end
         
    % Get polar patches based on ellipses
    polar_patches = cell(1,length(ellipses));
    for i = 1:length(ellipses)   
        if isempty(ellipses(i).x) || isempty(ellipses(i).y) || ...
           isempty(ellipses(i).r1) || isempty(ellipses(i).r2) || ...
           isempty(ellipses(i).rot)
            continue
        end
        
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
        polar_patch = alg.array_interp(array, p_affine', 'bicubic');
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
        % Get max value
        [i_max, j_max] = find(cc_mat == max(cc_mat(:)),1);
                     
        % Set coordinates
        four_points_p(j_max,:) = [ellipses(i_max).x ellipses(i_max).y];
                                       
        % "disable" this patch and template
        cc_mat(i_max,:) = -Inf; % patch
        cc_mat(:,j_max) = -Inf; % template
        
        % Store best patch matches
        patch_matches{i,1} = circshift(polar_patches{i_max},-(i_idx_mat(i_max,j_max)-1));
        patch_matches{i,2} = marker_templates.polar_patches{j_max}(:,j_idx_mat(i_max,j_max):j_idx_mat(i_max,j_max)+length(r_samples)-1);
    end        
end