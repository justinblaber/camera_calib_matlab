function [p_cb_ps, cov_cb_ps, idx_valid, debug] = refine_circle_points_cb_w2p(array_cb, f_p_cb_w2p_cb_p, opts)
    % Performs refinement of center of circle targets on a calibration
    % board image array.
    %
    % Inputs:
    %   array_cb - array; MxN array
    %   f_p_cb_w2p_cb_p - function handle; function which transforms
    %       calibration world points to calibration pixel points
    %   opts - struct;
    %       .cb_class - class.cb_pattern; calibration board object
    %       .refine_ellipse_edges_h2_init - scalar; initial value of h2
    %           parameter in "edges" ellipse refinement
    %       .refine_ellipse_edges_it_cutoff - int; max number of
    %           iterations performed for "edges" ellipse refinement
    %       .refine_ellipse_edges_norm_cutoff - scalar; cutoff for the
    %           difference in norm of the parameter vector for "edges"
    %           ellipse refinement
    %       .refine_circle_dualconic_edges_diff_norm_cutoff - scalar;
    %           cutoff for the norm of difference of the points predicted
    %           by dualconic and edges method
    %
    % Outputs:
    %   p_cb_ps - array; Px2 array of optimized circle pixel points
    %   cov_cb_ps - cell array; Px1 cell array of covariance matrices of
    %       circle pixel points
    %   idx_valid - array; Px1 logical array of "valid" circle points
    %   debug - cell array;

    % Get calibration board world points and boundaries
    p_cb_ws = opts.cb_class.get_p_cb_ws();
    boundary_ws = opts.cb_class.get_p_cb_w_boundaries();

    % Get array gradients
    array_dx = alg.grad_array(array_cb, 'x');
    array_dy = alg.grad_array(array_cb, 'y');

    % Cycle over points and refine them; also keep track of which indices
    % are "valid"
    p_cb_ps = zeros(size(p_cb_ws));
    cov_cb_ps = cell(size(p_cb_ws, 1), 1);
    idx_valid = false(size(p_cb_ws, 1), 1);
    debug = cell(size(p_cb_ws, 1), 1);
    for i = 1:size(p_cb_ws, 1)
        % Get calibration board world point and boundary
        p_cb_w = p_cb_ws(i, :);
        boundary_w = boundary_ws{i};

        % Get initial guess for calibration board pixel point
        p_cb_p_init = f_p_cb_w2p_cb_p(p_cb_w);

        % Get boundary in pixel coordinates centered around point
        boundary_p_center = f_p_cb_w2p_cb_p(boundary_w) - p_cb_p_init;

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
            debug{i}.e_cb_p_dualconic = e_cb_p_dualconic;
            debug{i}.e_cb_p_edges = e_cb_p_edges;
            debug{i}.boundary_p_edges = boundary_p_center + e_cb_p_dualconic(1:2)'; % Dual conic determines boundary offset for edges refinement
        end
    end
end

function [bb_p, mask] = calc_bb_and_mask(p_cb_p, boundary_p_center)
    % Add center point
    boundary_p = boundary_p_center + p_cb_p;

    % Get bounding box
    bb_p = [floor(min(boundary_p));
            ceil(max(boundary_p))];

    % Get mask
    mask = poly2mask(boundary_p(:, 1) - bb_p(1, 1) + 1, ...
                     boundary_p(:, 2) - bb_p(1, 2) + 1, ...
                     bb_p(2, 2)-bb_p(1, 2)+1, ...
                     bb_p(2, 1)-bb_p(1, 1)+1);
end

function e_cb_p = dualconic(p_cb_p_init, array_dx, array_dy, boundary_p_center)
    % Get bb of array
    bb_array_p = alg.bb_array(array_dx);

    % Get bounding box and mask of sub array
    [bb_sub_p, mask_sub] = calc_bb_and_mask(p_cb_p_init, boundary_p_center);

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        % An output with nans indicates that this process failed
        e_cb_p = nan(5, 1);
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

    % Apply masks
    sub_array_dx(~mask_sub) = nan;
    sub_array_dy(~mask_sub) = nan;

    % Fit ellipse using dual conic method; note that coordinates will be
    % WRT sub_array.
    e_cb_p_sub = alg.refine_ellipse_dualconic(sub_array_dx, sub_array_dy);

    % Get ellipse in array coordinates.
    e_cb_p = e_cb_p_sub;
    e_cb_p(1:2) = e_cb_p_sub(1:2) + bb_sub_p(1, :)' - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(e_cb_p(1:2)', bb_sub_p)
        e_cb_p = nan(5, 1);
        return
    end
end

function [e_cb_p, cov_cb_p, bb_sub_p] = edges(e_cb_p_init, array_dx, array_dy, boundary_p_center, opts)
    % Get bb of array
    bb_array_p = alg.bb_array(array_dx);

    % Get bounding box and mask of sub arrays
    [bb_sub_p, mask_sub] = calc_bb_and_mask(e_cb_p_init(1:2)', boundary_p_center);

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        % An output with nans indicates that this process failed
        e_cb_p = nan(5, 1);
        cov_cb_p = nan(2);
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

    % Apply masks
    sub_array_dx(~mask_sub) = nan;
    sub_array_dy(~mask_sub) = nan;

    % Get refined ellipse; note that coordinates will be WRT sub_array.
    e_cb_p_sub_init = e_cb_p_init;
    e_cb_p_sub_init(1:2) = e_cb_p_init(1:2) - bb_sub_p(1, :)' + 1;
    [e_cb_p_sub, cov_cb_e] = alg.refine_ellipse_edges(sub_array_dx, ...
                                                      sub_array_dy, ...
                                                      e_cb_p_sub_init, ...
                                                      opts);

    % Get covariance of center
    cov_cb_p = cov_cb_e(1:2, 1:2);

    % Get ellipse in array coordinates.
    e_cb_p = e_cb_p_sub;
    e_cb_p(1:2) = e_cb_p_sub(1:2) + bb_sub_p(1, :)' - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(e_cb_p(1:2)', bb_sub_p)
        e_cb_p = nan(5, 1);
        cov_cb_p = nan(2);
        return
    end
end
