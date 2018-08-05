function [board_points_p, idx_valid] = refine_points(array,homography_or_calib,opts)

    % Get forward xfm
    switch class(homography_or_calib)
        case 'struct'
            % Calibration
            
        case 'double'
            % Homography
            forward_xfm = @(p)(alg.apply_homography(homography_or_calib,p));
    end
    
    % Determine sampling rate by grabbing four corners, getting distances
    % along perimeter of the box, and then computing the max sampling rate.
    % Note:
    %   p1 - l2 - p3
    %    |         |
    %   l1        l4
    %    |         |
    %   p2 - l3 - p4
    board_points_w = alg.cb_points(opts);
    
    % Get points
    p1 = forward_xfm(board_points_w(1,:));
    p2 = forward_xfm(board_points_w(opts.num_targets_height,:));
    p3 = forward_xfm(board_points_w(end-(opts.num_targets_height-1),:));
    p4 = forward_xfm(board_points_w(end,:));
    
    % Get distances
    l1 = norm(p2 - p1);
    l2 = norm(p3 - p1);
    l3 = norm(p4 - p2);
    l4 = norm(p4 - p3);
    
    % Get pixels per target spacing - do approximately twice the max
    % distance and also ensure its an odd number so the center of the
    % target is sampled.
    pix_per_target_spacing = max([l1/opts.num_targets_height, ...
                                  l2/opts.num_targets_width, ...
                                  l3/opts.num_targets_width, ...
                                  l4/opts.num_targets_height]);
    pix_per_target_spacing = ceil(pix_per_target_spacing/2)*4+1;

    % Resample array to "frontal" space
    
    % Get frontal array size
    array_frontal_width = opts.num_targets_width * pix_per_target_spacing;
    array_frontal_height = opts.num_targets_height * pix_per_target_spacing;

    % Get coordinates of pixels of frontal array in world coordinates
    units_per_pix = opts.target_spacing/pix_per_target_spacing;

    % Get frontal height and width in world coordinates
    array_frontal_width_w = (array_frontal_width-1) * units_per_pix;
    array_frontal_height_w = (array_frontal_height-1) * units_per_pix;
    
    % Get world coordinates of bounding box of frontal image
    x_frontal_top_left_w = -(pix_per_target_spacing-1)/2*units_per_pix;
    y_frontal_top_left_w = -(pix_per_target_spacing-1)/2*units_per_pix;

    x_frontal_bottom_right_w = x_frontal_top_left_w + array_frontal_width_w;
    y_frontal_bottom_right_w = y_frontal_top_left_w + array_frontal_height_w;

    % Get world coordinates of pixels in frontal image
    [y_frontal_w, x_frontal_w] = ndgrid(linspace(y_frontal_top_left_w,y_frontal_bottom_right_w,array_frontal_height), ...
                                        linspace(x_frontal_top_left_w,x_frontal_bottom_right_w,array_frontal_width));

    % Apply transform to bring world coordinates into image pixel
    % coordinates
    p_frontal_p = forward_xfm([x_frontal_w(:) y_frontal_w(:)]);
                                    
    % Resample array
    array_frontal = alg.array_interp(array, ...
                                     p_frontal_p, ...
                                     'cubic');
    array_frontal = reshape(array_frontal,[array_frontal_height array_frontal_width]);
                                       
    % Get gradient images
    array_frontal_dx = alg.array_grad(array_frontal,'x');
    array_frontal_dy = alg.array_grad(array_frontal,'y');
        
    % Cycle over points and refine them; initialize valid indices
    board_points_p = zeros(size(board_points_w));
    idx_valid = false(size(board_points_w,1),1);
    for i = 1:opts.num_targets_height
        for j = 1:opts.num_targets_width
            % Get sub arrays
            sub_array_frontal_dx = array_frontal_dx((i-1)*pix_per_target_spacing+1:i*pix_per_target_spacing, ...
                                                    (j-1)*pix_per_target_spacing+1:j*pix_per_target_spacing);
            sub_array_frontal_dy = array_frontal_dy((i-1)*pix_per_target_spacing+1:i*pix_per_target_spacing, ...
                                                    (j-1)*pix_per_target_spacing+1:j*pix_per_target_spacing);
            
            % Make sure no pixels are NaNs which indicates the point is
            % nearly out of bounds
            if ~any(isnan(sub_array_frontal_dx(:))) && ~any(isnan(sub_array_frontal_dy(:)))
                % Get sub pixel point; note this will be in "frontal" coordinates              
                board_point_frontal_sub = alg.refine_checker(sub_array_frontal_dx, ...
                                                             sub_array_frontal_dy, ...
                                                             opts);
                                                         
                % Convert to frontal coordinates
                board_point_frontal_sub(1) = board_point_frontal_sub(1) + (j-1)*pix_per_target_spacing;
                board_point_frontal_sub(2) = board_point_frontal_sub(2) + (i-1)*pix_per_target_spacing;
                                                    
                % Convert to world coordinates
                board_point_w = (board_point_frontal_sub - (pix_per_target_spacing+1)/2)*units_per_pix;
                
                % Convert to pixel coordinates
                idx = (j-1)*opts.num_targets_height+i;
                board_points_p(idx,:) = forward_xfm(board_point_w);
                idx_valid(idx) = true;                
            end
        end
    end
    
end