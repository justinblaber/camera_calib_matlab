function [board_points_p, board_covs_p, idx_valid, debug] = refine_checker_points(array,xfm_w2p,opts)
    % Performs refinement of checker points on an array.
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
    %       .refine_checker_min_hw - int; min half window for checker
    %           refinement
    %       .refine_checker_max_hw - int; max half window for checker
    %           refinement
    %       .refine_checker_opencv_it_cutoff - int; max number of 
    %           iterations performed for "opencv" checker refinement
    %       .refine_checker_opencv_norm_cutoff - scalar; cutoff for the 
    %           difference in norm of center position for "opencv" checker
    %           refinement
    %       .refine_checker_edges_h2_init - scalar; initial value of h2
    %           parameter in "edges" checker refinement
    %       .refine_checker_edges_it_cutoff - int; max number of 
    %           iterations performed for "edges" checker refinement 
    %       .refine_checker_edges_norm_cutoff - scalar; cutoff for the 
    %           difference in norm of the parameter vector for "edges" 
    %           checker refinement
    %
    % Outputs:
    %   board_points_p - array; Px2 array of optimized subpixel checker
    %       points in pixel coordinates
    %   board_covs_p - cell array; Px1 cell array of covariance matrices 
    %       for detected corners
    %   idx_valid - array; Px1 logical array of "valid" checker points
    %   debug - cell array; Px1 cell array of bounding boxes of the
    %       refinement windows
    
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
        
        % Get half window of sub array
        hw_sub_array = get_hw_sub_array(p_w, ...
                                        xfm_w2p, ...
                                        opts);
        
        % Perform initial refinement with "opencv" checker detection. This
        % method is pretty robust and reasonably accurate.
        p_p_opencv = opencv(p_p_init, ...
                            array_dx, ...
                            array_dy, ...
                            hw_sub_array, ...
                            opts);
       
        % If "opencv" refinement failed, then p_p_opencv will be nans
        if any(isnan(p_p_opencv))
            continue
        end
        
        % Perform final "edges" refinement
        [p_p_edges, cov_p_edges, bb_sub_array_edges] = edges(p_p_opencv, ...
                                                             p_w, ...
                                                             xfm_w2p, ...
                                                             array_dx, ...
                                                             array_dy, ...
                                                             hw_sub_array, ...
                                                             opts);

        % If "edges" refinement failed, then p_p_edges or cov_p_edges will 
        % be nans
        if ~any(isnan(p_p_edges)) && ~any(isnan(cov_p_edges(:)))
            board_points_p(i,:) = p_p_edges;
            board_covs_p{i} = cov_p_edges;
            idx_valid(i) = true;
            debug{i} = bb_sub_array_edges; %#ok<AGROW>
        end
    end    
end

function hw_sub_array = get_hw_sub_array(p_w,xfm_w2p,opts)
    % Get box around point in world coordinates
    % Note:
    %    p1 - l2 - p3
    %    |         |
    %    l1   p    l4
    %    |         |
    %    p2 - l3 - p4
    box_w = [p_w(1)-opts.target_spacing p_w(2)-opts.target_spacing;
             p_w(1)-opts.target_spacing p_w(2)+opts.target_spacing;
             p_w(1)+opts.target_spacing p_w(2)-opts.target_spacing;
             p_w(1)+opts.target_spacing p_w(2)+opts.target_spacing];

    % Apply xform to go from world coordinates to pixel coordinates
    box_p = xfm_w2p(box_w);

    % Form lines in pixel coordinates
    l1_p = alg.points2line(box_p(1,:),box_p(2,:));
    l2_p = alg.points2line(box_p(1,:),box_p(3,:));
    l3_p = alg.points2line(box_p(2,:),box_p(4,:));
    l4_p = alg.points2line(box_p(3,:),box_p(4,:));

    % Get shortest distance from lines to target point in pixel
    % coordinates
    p_p = xfm_w2p(p_w);        
    d1 = alg.point_line_distance(p_p,l1_p);
    d2 = alg.point_line_distance(p_p,l2_p);
    d3 = alg.point_line_distance(p_p,l3_p);
    d4 = alg.point_line_distance(p_p,l4_p);

    % Get minimum distance
    d_min = min([d1 d2 d3 d4]);

    % Get half width of circumscribed square, which should help prevent
    % neighboring checkers from being inside the square.
    hw_sub_array = floor(d_min/sqrt(2));
    if hw_sub_array < opts.refine_checker_min_hw
        warning(['Minimum half width set, check to make sure checkers ' ...
                 'in this image are not too small.']);
        hw_sub_array = opts.refine_checker_min_hw;
    elseif hw_sub_array > opts.refine_checker_max_hw
        hw_sub_array = opts.refine_checker_max_hw;
    end
end

function p_p = opencv(p_p_init,array_dx,array_dy,hw_sub_array,opts)
    % Initialize
    p_p = p_p_init;

    % Perform iterations until convergence
    for it = 1:opts.refine_checker_opencv_it_cutoff            
        % Get bounding box of sub array
        p_p_rounded = round(p_p);
        bb_sub_array = [p_p_rounded(1)-hw_sub_array p_p_rounded(2)-hw_sub_array;
                        p_p_rounded(1)+hw_sub_array p_p_rounded(2)+hw_sub_array];

        % Check bounds
        if bb_sub_array(1,1) < 1 || bb_sub_array(2,1) > size(array_dx,2) || ...
           bb_sub_array(1,2) < 1 || bb_sub_array(2,2) > size(array_dx,1)
            % An output with nans indicates that this process failed
            p_p = nan(1,2);
            return
        end

        % Get sub arrays
        sub_array_dx = array_dx(bb_sub_array(1,2):bb_sub_array(2,2), ...
                                bb_sub_array(1,1):bb_sub_array(2,1));
        sub_array_dy = array_dy(bb_sub_array(1,2):bb_sub_array(2,2), ...
                                bb_sub_array(1,1):bb_sub_array(2,1));

        % Cache previous point
        p_prev = p_p;

        % Get refined point; note that coordinates will be WRT sub_array.
        p_p_sub = alg.refine_checker_opencv(sub_array_dx, ...
                                            sub_array_dy, ...
                                            p_p - bb_sub_array(1,:) + 1);

        % Get point in array coordinates.
        p_p = p_p_sub + bb_sub_array(1,:) - 1;    
        
        % Exit if change in distance is small
        diff_norm = norm(p_prev-p_p);
        if diff_norm < opts.refine_checker_opencv_norm_cutoff
            break
        end
    end
end

function [p_p, cov_p, bb_sub_array] = edges(p_p_init,p_w,xfm_w2p,array_dx,array_dy,hw_sub_array,opts)
    % Initialize
    p_p = p_p_init;
    
    % Get bounding box of sub array
    p_p_rounded = round(p_p);
    bb_sub_array = [p_p_rounded(1)-hw_sub_array p_p_rounded(2)-hw_sub_array;
                    p_p_rounded(1)+hw_sub_array p_p_rounded(2)+hw_sub_array];

    % Check bounds
    if bb_sub_array(1,1) < 1 || bb_sub_array(2,1) > size(array_dx,2) || ...
       bb_sub_array(1,2) < 1 || bb_sub_array(2,2) > size(array_dx,1)
        % An output with nans indicates that this process failed
        p_p = nan(1,2);
        cov_p = nan(2);
        return
    end

    % Get sub arrays
    sub_array_dx = array_dx(bb_sub_array(1,2):bb_sub_array(2,2), ...
                            bb_sub_array(1,1):bb_sub_array(2,1));
    sub_array_dy = array_dy(bb_sub_array(1,2):bb_sub_array(2,2), ...
                            bb_sub_array(1,1):bb_sub_array(2,1));

    % Get diamond around point in world coordinates
    % Note:
    %         p2
    %         |
    %    p1 - p - p4
    %         |
    %         p3
    diamond_w = [p_w(1)-opts.target_spacing p_w(2);
                 p_w(1) p_w(2)-opts.target_spacing;
                 p_w(1) p_w(2)+opts.target_spacing;
                 p_w(1)+opts.target_spacing p_w(2)];

    % Apply xform to go from world coordinates to pixel coordinates
    diamond_p = xfm_w2p(diamond_w);            

    % Get points in sub array coordinates
    diamond_p_sub = diamond_p - bb_sub_array(1,:) + 1;

    % Get initial line estimates
    l1_p = alg.pointslope2line(p_p - bb_sub_array(1,:) + 1, ...
                               (diamond_p_sub(4,2)-diamond_p_sub(1,2))/(diamond_p_sub(4,1)-diamond_p_sub(1,1)));
    l2_p = alg.pointslope2line(p_p - bb_sub_array(1,:) + 1, ...
                               (diamond_p_sub(3,2)-diamond_p_sub(2,2))/(diamond_p_sub(3,1)-diamond_p_sub(2,1)));

    % Get refined point; note that coordinates will be WRT
    % sub_array.
    [p_p_sub, cov_p] = alg.refine_checker_edges(sub_array_dx, ...
                                                sub_array_dy, ...
                                                l1_p, ...
                                                l2_p, ...
                                                opts);
    
    % Get point in array coordinates.
    p_p = p_p_sub + bb_sub_array(1,:) - 1;  
end