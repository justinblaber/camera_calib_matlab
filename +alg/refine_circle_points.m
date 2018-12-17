function [p_cb_ps, cov_cb_ps, idx_valid, debug] = refine_circle_points(array_cb, f_p_w2p_p, opts, idx_valid_init)
    % Performs refinement of center of circle targets on a calibration
    % board image array.
    %
    % Inputs:
    %   array_cb - array; MxN array
    %   f_p_w2p_p - function handle; function which transforms world
    %     coordinates to pixel coordinates
    %   opts - struct;
    %       .height_fp - scalar; height of the "four point" box
    %       .width_fp - scalar; width of the "four point" box
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
    %   idx_valid_init - array; logical indices which indicate which target
    %       points are valid
    %
    % Outputs:
    %   p_cb_ps - array; Px2 array of optimized subpixel ellipse points in
    %       pixel coordinates
    %   cov_cb_ps - cell array; Px1 cell array of covariance matrices of
    %       ellipse points
    %   idx_valid - array; Px1 logical array of "valid" ellipse points
    %   debug - cell array; Px1 cell array of structs containing:
    %       .e_cb_p_dualconic - array; 5x1 array of ellipse found using
    %           "dual conic" method
    %       .e_cb_p_edges - array; 5x1 array of ellipse found using "edges"
    %           method
    %       .boundary_p - array; 4x2 array of boundary points used in
    %           "edges" method

    if ~exist('idx_valid_init', 'var')
        idx_valid_init = true(opts.num_targets_height*opts.num_targets_width, 1);
    end

    % Get board points in world coordinates
    p_cb_ws = alg.p_cb_w(opts);

    % Get array gradients
    array_dx = alg.grad_array(array_cb, 'x');
    array_dy = alg.grad_array(array_cb, 'y');

    % Cycle over points and refine them; also keep track of which indices
    % are "valid"
    p_cb_ps = zeros(size(p_cb_ws));
    cov_cb_ps = cell(size(p_cb_ws, 1), 1);
    idx_valid = false(size(p_cb_ws, 1), 1);
    for i = 1:size(p_cb_ws, 1)
        if ~idx_valid_init(i)
            continue
        end

        % Get point in world coordinates
        p_cb_w = p_cb_ws(i, :);

        % Convert point to pixel coordinates to get initial guess
        p_cb_p_init = f_p_w2p_p(p_cb_w);

        % Get boundary in pixel coordinates centered around point
        boundary_p_center = calc_boundary(p_cb_w, ...
                                          f_p_w2p_p, ...
                                          opts);

        % Perform initial refinement with "dual conic" ellipse detection.
        e_cb_p_dualconic = dualconic(p_cb_p_init, ...
                                     array_dx, ...
                                     array_dy, ...
                                     boundary_p_center);

        % Make sure e_cb_p_dualconic is valid
        if any(~isfinite(e_cb_p_dualconic))
            continue
        end

        % Perform final "edges" refinement
        [e_cb_p_edges, cov_cb_p_edges] = edges(e_cb_p_dualconic, ...
                                               array_dx, ...
                                               array_dy, ...
                                               boundary_p_center, ...
                                               opts);

        % Make sure:
        % 1) e_cb_p_edges and cov_cb_p_edges are valid
        % 2) cov_cb_p_edges is positive definite. lscov() for sparse
        %   matrices requires covariance matrix to be positive definite. If
        %   the model is perfect, then covariance will be zero, but this is
        %   basically an impossibility with real data.
        % 3) points from dual conic and edges estimation are close; if not,
        %   this means this ellipse is unreliable
        if all(isfinite(e_cb_p_edges)) && ...
           all(isfinite(cov_cb_p_edges(:))) && ...
           alg.is_pos_def(cov_cb_p_edges) && ...
           norm(e_cb_p_dualconic(1:2)' - e_cb_p_edges(1:2)') < opts.refine_circle_dualconic_edges_diff_norm_cutoff
            p_cb_ps(i, :) = e_cb_p_edges(1:2)';
            cov_cb_ps{i} = cov_cb_p_edges;
            idx_valid(i) = true;
            debug{i}.e_cb_p_dualconic = e_cb_p_dualconic; %#ok<AGROW>
            debug{i}.e_cb_p_edges = e_cb_p_edges; %#ok<AGROW>
            debug{i}.boundary_p = boundary_p_center + e_cb_p_dualconic(1:2)'; %#ok<AGROW>
        end
    end
end

function boundary_p_center = calc_boundary(p_w, f_p_w2p_p, opts)
    % Get boundary around point in world coordinates
    % Note:
    %    p1 ----- p4
    %    |         |
    %    |    p    |
    %    |         |
    %    p2 ----- p3
    boundary_w = [p_w(1)-opts.target_spacing/2 p_w(2)-opts.target_spacing/2;
                  p_w(1)-opts.target_spacing/2 p_w(2)+opts.target_spacing/2;
                  p_w(1)+opts.target_spacing/2 p_w(2)+opts.target_spacing/2;
                  p_w(1)+opts.target_spacing/2 p_w(2)-opts.target_spacing/2];

    % Apply xform to go from world coordinates to pixel coordinates
    boundary_p = f_p_w2p_p(boundary_w);

    % Subtract center point
    p_p = f_p_w2p_p(p_w);
    boundary_p_center = boundary_p - p_p;
end

function [bb_p, mask] = calc_bb_and_mask(p_p, boundary_p_center)
    % Get boundary in pixel coordinates
    boundary_p = boundary_p_center + p_p;

    % Get bounding box
    bb_p = [floor(min(boundary_p));
            ceil(max(boundary_p))];

    % Get mask
    mask = poly2mask(boundary_p(:, 1) - bb_p(1, 1) + 1, ...
                     boundary_p(:, 2) - bb_p(1, 2) + 1, ...
                     bb_p(2, 2)-bb_p(1, 2)+1, ...
                     bb_p(2, 1)-bb_p(1, 1)+1);
end

function e_p = dualconic(p_p_init, array_dx, array_dy, boundary_p_center)
    % Get bb of array
    bb_array = alg.bb_array(array_dx);

    % Get bounding box and mask of sub array
    [bb_p_sub, mask_sub_array] = calc_bb_and_mask(p_p_init, boundary_p_center);

    % Check bounds
    if ~alg.is_bb_in_bb(bb_p_sub, bb_array)
        % An output with nans indicates that this process failed
        e_p = nan(5, 1);
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_dx, bb_p_sub);
    sub_array_dy = alg.get_sub_array_bb(array_dy, bb_p_sub);

    % Apply masks
    sub_array_dx(~mask_sub_array) = nan;
    sub_array_dy(~mask_sub_array) = nan;

    % Fit ellipse using dual conic method; note that coordinates will be
    % WRT sub_array.
    e_p_sub = alg.refine_ellipse_dualconic(sub_array_dx, sub_array_dy);

    % Get ellipse in array coordinates.
    e_p = e_p_sub;
    e_p(1:2) = e_p_sub(1:2)' + bb_p_sub(1, :) - 1;

    % Make sure p_p did not go outside of original window
    if ~alg.is_p_in_bb(e_p(1:2)', bb_p_sub)
        e_p = nan(5, 1);
        return
    end
end

function [e_p, cov_p, bb_p_sub] = edges(e_p_init, array_dx, array_dy, boundary_p_center, opts)
    % Get bb of array
    bb_array = alg.bb_array(array_dx);

    % Get bounding box and mask of sub arrays
    [bb_p_sub, mask_sub_array] = calc_bb_and_mask(e_p_init(1:2)', boundary_p_center);

    % Check bounds
    if ~alg.is_bb_in_bb(bb_p_sub, bb_array)
        % An output with nans indicates that this process failed
        e_p = nan(5, 1);
        cov_p = nan(2);
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_dx, bb_p_sub);
    sub_array_dy = alg.get_sub_array_bb(array_dy, bb_p_sub);

    % Apply masks
    sub_array_dx(~mask_sub_array) = nan;
    sub_array_dy(~mask_sub_array) = nan;

    % Get refined ellipse; note that coordinates will be WRT sub_array.
    e_p_sub_init = e_p_init;
    e_p_sub_init(1:2) = e_p_init(1:2) - bb_p_sub(1, :)' + 1;
    [e_p_sub, cov_e] = alg.refine_ellipse_edges(sub_array_dx, ...
                                                sub_array_dy, ...
                                                e_p_sub_init, ...
                                                opts);

    % Get covariance of center
    cov_p = cov_e(1:2, 1:2);

    % Get ellipse in array coordinates.
    e_p = e_p_sub;
    e_p(1:2) = e_p_sub(1:2) + bb_p_sub(1, :)' - 1;

    % Make sure p_p did not go outside of original window
    if ~alg.is_p_in_bb(e_p(1:2)', bb_p_sub)
        e_p = nan(5, 1);
        cov_p = nan(2);
        return
    end
end
