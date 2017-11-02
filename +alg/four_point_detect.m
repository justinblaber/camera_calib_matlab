function four_points_p = four_point_detect(array,calib_config)
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
    
    % Read marker config and marker templates
    marker_config = util.read_data(calib_config.marker_config_path);
    marker_templates = util.read_data(calib_config.marker_templates_path);
    
    % Make sure there are four marker templates
    if ~isfield(marker_templates,'polar_patches') || length(marker_templates.polar_patches) ~= 4
        error('There must be four templates in the marker templates file!');
    end
    
    % Get normalized radius and theta samples
    r_samples = linspace(marker_config.radius_norm_range(1), ...
                         marker_config.radius_norm_range(2), ...
                         marker_config.radius_num_samples);
    % For theta, sample 1 more and then remove it; this allows [0 2*pi)
    theta_samples = linspace(0,2*pi,marker_config.theta_num_samples+1);
    theta_samples(end) = []; 
    
    % Get blobs
    blobs = alg.blob_detect(array,calib_config);
    
    % Get k
    k = 2^(1/calib_config.blob_detect_s);
    
    % Get polar patches of blobs
    polar_patches = cell(1,length(blobs));
    for i = 1:length(blobs)
        % Get eigenvalue decomposition of M
        [V,D] = eig(blobs(i).M);
        
        % Ellipse sides are proportional to sqrt of eigenvalues
        r_pix = sqrt(2)*(1+(k-1)/2)*blobs(i).sigma;             % radius of blob in pixels
        scale_factor = 2*r_pix/(sqrt(D(1,1))+sqrt(D(2,2)));     % have minor and major axis sum to diameter of blob; not sure if this is correct/ideal
        r1_pix = sqrt(D(1,1))*scale_factor;                     % major axis radius
        r2_pix = sqrt(D(2,2))*scale_factor;                     % minor axis radius
        
        % Rotation of major axis
        rot = -atan2(V(1,1),V(2,1));                            
        
        % Make xform to apply to coordinates of circle: rotation * scaling
        xform = [cos(rot) -sin(rot); sin(rot) cos(rot)] * [r1_pix 0; 0 r2_pix];
                             
        % Get normalized coordinates
        x = bsxfun(@times,cos(theta_samples)',r_samples); 
        y = bsxfun(@times,sin(theta_samples)',r_samples); 
        
        % Apply transformation and translate to center of blob
        p_affine = xform * vertcat(x(:)',y(:)');
        p_affine(1,:) = p_affine(1,:)+blobs(i).x;
        p_affine(2,:) = p_affine(2,:)+blobs(i).y;
        
        % Check for out of bounds points
        idx_out = p_affine(1,:) < 1 | p_affine(1,:) > size(array,2) | ...
                  p_affine(2,:) < 1 | p_affine(2,:) > size(array,1);              
        if any(idx_out)
            % Out of bounds sampling means matching won't work properly
            % anyway, so just move on to next point
            continue
        end
                        
        % Resample
        polar_patch = alg.array_interp(array, ...
                                       vertcat(p_affine(1,:),p_affine(2,:))', ...
                                       'bicubic');
        polar_patches{i} = reshape(polar_patch,marker_config.theta_num_samples,[]);     
    end
    
    % Normalize template patches
    for i = 1:4
        marker_templates.polar_patches{i} = marker_templates.polar_patches{i} - mean(marker_templates.polar_patches{i}(:));
        marker_templates.polar_patches{i} = marker_templates.polar_patches{i}./norm(marker_templates.polar_patches{i}(:));
    end
    
    % Cross correlate each polar patch with 4 templates. 
    % Initialize to -1, which is the min value for normalized cross correlation
    cc_mat = -ones(length(polar_patches),4); 
    for i = 1:length(polar_patches)
        % Make sure polar patch isn't empty, which can happen if sampling
        % points are outside image field of view. continue'ing is safe
        % because the cross correlation value will be negative 1 by default
        if isempty(polar_patches{i})
            continue
        end
        
        % Normalize polar patch
        polar_patches{i} = polar_patches{i} - mean(polar_patches{i}(:));
        polar_patches{i} = polar_patches{i}./norm(polar_patches{i}(:));
        
        % Compare to four templates with cross correlation
        for j = 1:4
            % Just do cross correlation over theta
            cc_buf = zeros(size(polar_patches{i},1),1);
            for m = 1:size(polar_patches{i},2)
                cc_buf = cc_buf + ifft(fft(polar_patches{i}(:,m)).*conj(fft(marker_templates.polar_patches{j}(:,m))));
            end
            % Store in cc_mat
            cc_mat(i,j) = max(cc_buf);
        end
    end   
    
    % Get best matches
    four_points_p = zeros(4,2);
    for i = 1:4
        % Get max value
        [i_max, j_max] = find(cc_mat == max(cc_mat(:)));
        
        % Set coordinates
        four_points_p(j_max,:) = [blobs(i_max).x blobs(i_max).y];
        
        % "disable" this blob and this marker
        cc_mat(i_max,:) = -1; % blob
        cc_mat(:,j_max) = -1; % marker
    end        
end