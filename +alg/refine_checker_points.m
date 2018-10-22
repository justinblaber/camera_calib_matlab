function [p_cb_ps, cov_cb_ps, idx_valid, debug] = refine_checker_points(array_cb,f_p_w2p_p,opts,idx_valid_init)
    % Performs refinement of center of checker targets on a calibration
    % board image array.
    %
    % Inputs:
    %   array_cb - array; MxN array
    %   f_p_w2p_p - function handle; function which transforms world
    %   	coordinates to pixel coordinates
    %   opts - struct;
    %       .height_fp - scalar; height of the "four point" box
    %       .width_fp - scalar; width of the "four point" box
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
    %   idx_valid_init - array; logical indices which indicate which target 
    %       points are valid 
    %
    % Outputs:
    %   p_cb_ps - array; Px2 array of optimized subpixel checker points in
    %       pixel coordinates
    %   cov_cb_ps - cell array; Px1 cell array of covariance matrices of
    %       checker points
    %   idx_valid - array; Px1 logical array of "valid" checker points
    %   debug - cell array; Px1 cell array of bounding boxes of the
    %       refinement windows
    
    if ~exist('idx_valid_init','var')
        idx_valid_init = true(opts.num_targets_height*opts.num_targets_width,1);
    end
    
    % Get board points in world coordinates
    p_cb_ws = alg.p_cb_w(opts);
    
    % Get array gradients
    array_dx = alg.grad_array(array_cb,'x');
    array_dy = alg.grad_array(array_cb,'y');

    % Cycle over points and refine them; also keep track of which indices
    % are "valid"
    p_cb_ps = zeros(size(p_cb_ws));
    cov_cb_ps = cell(size(p_cb_ws,1),1);
    idx_valid = false(size(p_cb_ws,1),1);
    for i = 1:size(p_cb_ws,1)
        if ~idx_valid_init(i)
            continue
        end
        
        % Get point in world coordinates
        p_cb_w = p_cb_ws(i,:);
        
        % Convert point to pixel coordinates to get initial guess
        p_cb_p_init = f_p_w2p_p(p_cb_w);
        
        % Get half window of sub array
        hw_p = calc_half_window(p_cb_w, ...
                                f_p_w2p_p, ...
                                opts);
        
        % Perform initial refinement with "opencv" checker detection.
        p_cb_p_opencv = opencv(p_cb_p_init, ...
                               array_dx, ...
                               array_dy, ...
                               hw_p, ...
                               opts);
       
        % If "opencv" refinement failed, then p_cb_p_opencv will be nans
        if any(isnan(p_cb_p_opencv))
            continue
        end
        
        % Perform final "edges" refinement
        [p_cb_p_edges, cov_cb_p_edges, bb_p_sub_edges] = edges(p_cb_p_opencv, ...
                                                               p_cb_w, ...
                                                               f_p_w2p_p, ...
                                                               array_dx, ...
                                                               array_dy, ...
                                                               hw_p, ...
                                                               opts);

        % If "edges" refinement failed, then p_cb_p_edges or cov_cb_p_edges
        % will be nans
        if ~any(isnan(p_cb_p_edges)) && ~any(isnan(cov_cb_p_edges(:)))
            p_cb_ps(i,:) = p_cb_p_edges;
            cov_cb_ps{i} = cov_cb_p_edges;
            idx_valid(i) = true;
            debug{i} = bb_p_sub_edges; %#ok<AGROW>
        end
    end    
end

function hw_p = calc_half_window(p_w,f_p_w2p_p,opts)
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
    box_p = f_p_w2p_p(box_w);

    % Form lines in pixel coordinates
    l1_p = alg.points2line(box_p(1,:),box_p(2,:));
    l2_p = alg.points2line(box_p(1,:),box_p(3,:));
    l3_p = alg.points2line(box_p(2,:),box_p(4,:));
    l4_p = alg.points2line(box_p(3,:),box_p(4,:));

    % Get shortest distance from lines to target point in pixel
    % coordinates
    p_p = f_p_w2p_p(p_w);        
    d1_p = alg.point_line_distance(p_p,l1_p);
    d2_p = alg.point_line_distance(p_p,l2_p);
    d3_p = alg.point_line_distance(p_p,l3_p);
    d4_p = alg.point_line_distance(p_p,l4_p);

    % Get minimum distance
    d_p_min = min([d1_p d2_p d3_p d4_p]);

    % Get half width of circumscribed square, which should help prevent
    % neighboring checkers from being inside the square.
    hw_p = floor(d_p_min/sqrt(2));
    if hw_p < opts.refine_checker_min_hw
        warning(['Minimum half width set, check to make sure checkers ' ...
                 'in this image are not too small.']);
        hw_p = opts.refine_checker_min_hw;
    elseif hw_p > opts.refine_checker_max_hw
        hw_p = opts.refine_checker_max_hw;
    end
end

function p_p = opencv(p_p_init,array_dx,array_dy,hw_p,opts)
    % Initialize
    p_p = p_p_init;

    % Perform iterations until convergence
    for it = 1:opts.refine_checker_opencv_it_cutoff            
        % Get bounding box of sub array
        p_p_rounded = round(p_p);
        bb_p_sub = [p_p_rounded(1)-hw_p p_p_rounded(2)-hw_p;
                    p_p_rounded(1)+hw_p p_p_rounded(2)+hw_p];

        % Check bounds
        if bb_p_sub(1,1) < 1 || bb_p_sub(2,1) > size(array_dx,2) || ...
           bb_p_sub(1,2) < 1 || bb_p_sub(2,2) > size(array_dx,1)
            % An output with nans indicates that this process failed
            p_p = nan(1,2);
            return
        end

        % Get sub arrays
        sub_array_dx = array_dx(bb_p_sub(1,2):bb_p_sub(2,2), ...
                                bb_p_sub(1,1):bb_p_sub(2,1));
        sub_array_dy = array_dy(bb_p_sub(1,2):bb_p_sub(2,2), ...
                                bb_p_sub(1,1):bb_p_sub(2,1));

        % Cache previous point
        p_p_prev = p_p;

        % Get refined point; note that coordinates will be WRT sub array.
        p_p_sub = alg.refine_checker_opencv(sub_array_dx, ...
                                            sub_array_dy, ...
                                            p_p - bb_p_sub(1,:) + 1);

        % Get point in array coordinates.
        p_p = p_p_sub + bb_p_sub(1,:) - 1;    
        
        % Exit if change in distance is small
        diff_norm = norm(p_p_prev-p_p);
        if diff_norm < opts.refine_checker_opencv_norm_cutoff
            break
        end
    end
end

function [p_p, cov_p, bb_p_sub] = edges(p_p_init,p_w,f_p_w2p_p,array_dx,array_dy,hw_p,opts)
    % Initialize
    p_p = p_p_init;
    
    % Get bounding box of sub array
    p_p_rounded = round(p_p);
    bb_p_sub = [p_p_rounded(1)-hw_p p_p_rounded(2)-hw_p;
                p_p_rounded(1)+hw_p p_p_rounded(2)+hw_p];

    % Check bounds
    if bb_p_sub(1,1) < 1 || bb_p_sub(2,1) > size(array_dx,2) || ...
       bb_p_sub(1,2) < 1 || bb_p_sub(2,2) > size(array_dx,1)
        % An output with nans indicates that this process failed
        p_p = nan(1,2);
        cov_p = nan(2);
        return
    end

    % Get sub arrays
    sub_array_dx = array_dx(bb_p_sub(1,2):bb_p_sub(2,2), ...
                            bb_p_sub(1,1):bb_p_sub(2,1));
    sub_array_dy = array_dy(bb_p_sub(1,2):bb_p_sub(2,2), ...
                            bb_p_sub(1,1):bb_p_sub(2,1));

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
    diamond_p = f_p_w2p_p(diamond_w);            

    % Get points in sub array coordinates
    diamond_p_sub = diamond_p - bb_p_sub(1,:) + 1;

    % Get initial line estimates
    l1_p_sub = alg.pointslope2line(p_p - bb_p_sub(1,:) + 1, ...
                                   (diamond_p_sub(4,2)-diamond_p_sub(1,2))/(diamond_p_sub(4,1)-diamond_p_sub(1,1)));
    l2_p_sub = alg.pointslope2line(p_p - bb_p_sub(1,:) + 1, ...
                                   (diamond_p_sub(3,2)-diamond_p_sub(2,2))/(diamond_p_sub(3,1)-diamond_p_sub(2,1)));

    % Get refined point; note that coordinates will be WRT sub_array.
    [p_p_sub, cov_p] = alg.refine_checker_edges(sub_array_dx, ...
                                                sub_array_dy, ...
                                                l1_p_sub, ...
                                                l2_p_sub, ...
                                                opts);
    
    % Get point in array coordinates.
    p_p = p_p_sub + bb_p_sub(1,:) - 1;  
end
