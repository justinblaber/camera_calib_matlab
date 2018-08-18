function [board_points_p, board_covs_p, idx_valid, debug] = refine_circle_points(array,xfm_w2p,opts)
    % Performs refinement of center of circle targets on an array.
    %
    % Inputs:
    %   array - array; MxN array
    %   xfm_w2p - function handle; function which transforms world
    %   	coordinates to pixel coordinates
    %   opts - struct;
    %       .four_point_height - scalar; height of the "four point" box
    %       .four_point_width - scalar; width of the "four point" box
    %       .num_targets_height - int; number of targets in the "height" 
    %           dimension
    %       .num_targets_width - int; number of targets in the "width"
    %           dimension
    %       .target_spacing - scalar; space between targets
    %       .refine_ellipse_edges_h2_init - scalar; initial value of h2
    %           parameter in "edges" ellipse refinement
	%       .refine_ellipse_edges_it_cutoff - int; max number of 
    %           iterations performed for "edges" ellipse refinement 
    %       .refine_ellipse_edges_norm_cutoff - scalar; cutoff for the 
    %           difference in norm of the parameter vector for "edges" 
    %           ellipse refinement
    %
    % Outputs:
    %   board_points_p - array; Px2 array of optimized subpixel ellipse
    %       points in pixel coordinates
    %   board_covs_p - cell array; Px1 cell array of covariance matrices 
    %       for detected ellipses
    %   idx_valid - array; Px1 logical array of "valid" checker points
    %   debug - cell array; Px1 cell array of structs containing:
    %       .e_p_dualconic - array; 5x1 array of ellipse found using 
    %           "dual conic" method
    %       .e_p_edges - array; 5x1 array of ellipse found using "edges" 
    %           method
    %       .boundary - array; 4x2 array of boundary points used in "edges"
    %           method
    
    % Get board points in world coordinates
    board_points_w = alg.cb_points(opts);
    
    % Get array gradients
    array_dx = alg.array_grad(array,'x');
    array_dy = alg.array_grad(array,'y');

    % Cycle over points and refine them; also keep track of which indices
    % are "valid"
    board_points_p = zeros(size(board_points_w));
    board_covs_p = cell(size(board_points_w,1),1);
    idx_valid = false(size(board_points_w,1),1);
    for i = 1:size(board_points_w,1)
        % Get point in world coordinates
        p_w = board_points_w(i,:);
        
        % Convert point to pixel coordinates to get initial guess
        p_p_init = xfm_w2p(p_w);
        
        % Get four point box in pixel coordinates centered around point
        box_center = get_box_center(p_w, ...
                                    xfm_w2p, ...
                                    opts);
        
        % Perform initial refinement with "dual conic" ellipse detection.
        e_p_dualconic = dualconic(p_p_init, ...
                                  array_dx, ...
                                  array_dy, ...
                                  box_center);
                     
        % If "opencv" refinement failed, then p_p_opencv will be nans
        if any(isnan(e_p_dualconic))
            continue
        end
                     
        % Perform final "edges" refinement
        [e_p_edges, cov_p_edges] = edges(e_p_dualconic, ...
                                         array_dx, ...
                                         array_dy, ...
                                         box_center, ...
                                         opts);
                                                                              
        % If "edges" refinement failed, then e_p_edges or cov_p_edges will 
        % be nans
        if ~any(isnan(e_p_edges)) && ~any(isnan(cov_p_edges(:)))
            board_points_p(i,:) = e_p_edges(1:2);
            board_covs_p{i} = cov_p_edges;
            idx_valid(i) = true;
            debug{i}.e_p_dualconic = e_p_dualconic; %#ok<AGROW>
            debug{i}.e_p_edges = e_p_edges; %#ok<AGROW>
            debug{i}.boundary = box_center + e_p_dualconic(1:2)'; %#ok<AGROW>
        end
    end    
end

function box_center = get_box_center(p_w,xfm_w2p,opts)
    % Get box around point in world coordinates
    % Note:
    %    p1 - l2 - p3
    %    |         |
    %    l1   p    l4
    %    |         |
    %    p2 - l3 - p4
    box_w = [p_w(1)-opts.target_spacing/2 p_w(2)-opts.target_spacing/2;
             p_w(1)-opts.target_spacing/2 p_w(2)+opts.target_spacing/2;
             p_w(1)+opts.target_spacing/2 p_w(2)-opts.target_spacing/2;
             p_w(1)+opts.target_spacing/2 p_w(2)+opts.target_spacing/2];

    % Apply xform to go from world coordinates to pixel coordinates
    box_p = xfm_w2p(box_w);
    
    % Subtract center point
    p_p = xfm_w2p(p_w);
    box_center = box_p - p_p;
end

function [bb, mask] = get_bb_and_mask(p_p, box_center)
    % Get box in pixel coordinates
    box_p = box_center + p_p;
    
    % Get bounding box
    bb = [floor(min(box_p(:,1))) floor(min(box_p(:,2)));
          ceil(max(box_p(:,1))) ceil(max(box_p(:,2)))];
    
    % Get mask
    mask = poly2mask(box_p([1 2 4 3],1) - bb(1,1) + 1, ...
                     box_p([1 2 4 3],2) - bb(1,2) + 1, ...
                     bb(2,2)-bb(1,2)+1, ...
                     bb(2,1)-bb(1,1)+1);    
end

function e_p = dualconic(p_p_init,array_dx,array_dy,box_center)    
    % Get bounding box and mask of sub array
    [bb_sub_array, mask_sub_array] = get_bb_and_mask(p_p_init, box_center);

    % Check bounds
    if bb_sub_array(1,1) < 1 || bb_sub_array(2,1) > size(array_dx,2) || ...
       bb_sub_array(1,2) < 1 || bb_sub_array(2,2) > size(array_dx,1)
        % An output with nans indicates that this process failed
        e_p = nan(1,5);
        return
    end

    % Get sub arrays
    sub_array_dx = array_dx(bb_sub_array(1,2):bb_sub_array(2,2), ...
                            bb_sub_array(1,1):bb_sub_array(2,1));
    sub_array_dy = array_dy(bb_sub_array(1,2):bb_sub_array(2,2), ...
                            bb_sub_array(1,1):bb_sub_array(2,1));
                        
    % Apply masks
    sub_array_dx(~mask_sub_array) = 0;
    sub_array_dy(~mask_sub_array) = 0;

    % Fit ellipse using dual conic method; note that coordinates will be 
    % WRT sub_array.
    e_p_sub = alg.refine_ellipse_dualconic(sub_array_dx, sub_array_dy);
    
    % Get ellipse in array coordinates.
    e_p = e_p_sub;
    e_p(1:2) = e_p_sub(1:2)' + bb_sub_array(1,:) - 1;    
end

function [e_p, cov_p, bb_sub_array] = edges(e_p_init,array_dx,array_dy,box_center,opts)
    % Get bounding box and mask of sub arrays
    [bb_sub_array, mask_sub_array] = get_bb_and_mask(e_p_init(1:2)', box_center);

    % Check bounds
    if bb_sub_array(1,1) < 1 || bb_sub_array(2,1) > size(array_dx,2) || ...
       bb_sub_array(1,2) < 1 || bb_sub_array(2,2) > size(array_dx,1)
        % An output with nans indicates that this process failed
        e_p = nan(1,5);
        cov_p = nan(2);
        return
    end

    % Get sub arrays
    sub_array_dx = array_dx(bb_sub_array(1,2):bb_sub_array(2,2), ...
                            bb_sub_array(1,1):bb_sub_array(2,1));
    sub_array_dy = array_dy(bb_sub_array(1,2):bb_sub_array(2,2), ...
                            bb_sub_array(1,1):bb_sub_array(2,1));
    
    % Apply masks
    sub_array_dx(~mask_sub_array) = 0;
    sub_array_dy(~mask_sub_array) = 0;
    
    % Get refined ellipse; note that coordinates will be WRT sub_array.
    e_p_sub_init = e_p_init;
    e_p_sub_init(1:2) = e_p_init(1:2)' - bb_sub_array(1,:) + 1;   
    [e_p_sub, cov_e] = alg.refine_ellipse_edges(sub_array_dx, ...
                                                sub_array_dy, ...
                                                e_p_sub_init, ...
                                                opts);
    
    % Get covariance of center
    cov_p = cov_e(1:2,1:2);
    
    % Get ellipse in array coordinates.
    e_p = e_p_sub;
    e_p(1:2) = e_p_sub(1:2)' + bb_sub_array(1,:) - 1;
end