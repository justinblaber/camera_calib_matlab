function [p_p, cov_p, debug] = refine_ellipse_point(p_p_init, boundary_p, array, array_dx, array_dy, opts)
    % Performs refinement of an ellipse target on an array.
    %
    % Inputs:
    %   p_p_init - array; 1x2 initial guess for target pixel point
    %   boundary_p - array; boundary points around target point in
    %       pixel coordinates
    %   array - array; MxN array
    %   array_dx - array; MxN gradient array in x direction
    %   array_dy - array; MxN gradient array in y direction
    %   opts - struct;
    %       .target_optimization - string; optimization type for target
    %       .* - options for f_refine_target_point() function
    %
    % Outputs:
    %   p_p - array; 1x2 optimized ellipse pixel point
    %   cov_p - array; 2x2 covariance matrix of ellipse pixel point
    %   debug - struct;

    % Run optimization ---------------------------------------------------%

    switch opts.target_optimization
        case 'dualconic'
            f_refine_target_point = @dualconic;
        case 'edges'
            f_refine_target_point = @edges;
        case 'dot'
            f_refine_target_point = @dot;
        otherwise
            error(['Unsupported target optimization: "' opts.target_optimization '" for ellipse target.']);
    end

    % Get ellipse
    [e_p, cov_p] = f_refine_target_point(p_p_init, ...
                                         boundary_p - p_p_init, ... % Center boundary
                                         array, ...
                                         array_dx, ...
                                         array_dy, ...
                                         opts);

    % Parse point
    p_p = e_p(1:2)';

    % Set debugging output
    debug.e_p = e_p;
    debug.boundary_p = boundary_p;
end

function [bb_p, mask] = calc_bb_and_mask(boundary_p)
    % Make sure all values are finite
    if any(~isfinite(boundary_p(:)))
        bb_p = nan(2);
        mask = [];
        return
    end

    % Get bounding box
    bb_p = [floor(min(boundary_p));
            ceil(max(boundary_p))];

    % Get mask
    mask = poly2mask(boundary_p(:, 1) - bb_p(1, 1) + 1, ...
                     boundary_p(:, 2) - bb_p(1, 2) + 1, ...
                     bb_p(2, 2)-bb_p(1, 2)+1, ...
                     bb_p(2, 1)-bb_p(1, 1)+1);
end

function [e_p, cov_p] = dualconic(p_p_init, boundary_p_centered, array, array_dx, array_dy, opts)
    % Initialize ellipse and covariance
    e_p = [p_p_init nan(1, 3)]';
    cov_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array);

    % Get initial bounding box of sub array
    bb_sub_p_init = calc_bb_and_mask(boundary_p_centered + p_p_init);

    % Perform iterations until convergence
    for it = 1:opts.refine_ellipse_dualconic_it_cutoff
        % Cache previous ellipse
        e_p_prev = e_p;

        % Get bounding box and mask of sub array
        [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_centered + e_p(1:2)');

        % Check bounds
        if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
            e_p(:) = nan;
            cov_p(:) = nan;
            return
        end

        % Get sub arrays
        sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
        sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

        % Fit ellipse using dual conic method; note that coordinates will be
        % WRT sub_array.
        e_p_sub = alg.refine_ellipse_dualconic(sub_array_dx, ...
                                               sub_array_dy, ...
                                               double(mask_sub));

        % TODO: update refine_ellipse_dualconic to return covariance of
        % ellipse center; for now just return an identity matrix.
        cov_p = eye(2);

        % Get ellipse in array coordinates.
        e_p = e_p_sub;
        e_p(1:2) = e_p(1:2) + bb_sub_p(1, :)' - 1;

        % Make sure point did not go outside of original bounding box
        if ~alg.is_p_in_bb(e_p(1:2)', bb_sub_p_init)
            e_p(:) = nan;
            cov_p(:) = nan;
            return
        end

        % Exit if change in distance is small
        diff_norm = norm(e_p_prev - e_p);
        if diff_norm < opts.refine_ellipse_dualconic_norm_cutoff
            break
        end
    end
end

function [e_p, cov_p] = edges(p_p_init, boundary_p_centered, array, array_dx, array_dy, opts)
    % Initialize ellipse by using dualconic method
    e_p = dualconic(p_p_init, boundary_p_centered, array, array_dx, array_dy, opts);
    cov_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array);

    % Get bounding box and mask of sub arrays
    [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_centered + e_p(1:2)');

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        e_p(:) = nan;
        cov_p(:) = nan;
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

    % Get refined ellipse; note that coordinates will be WRT sub_array.
    e_p_sub = e_p;
    e_p_sub(1:2) = e_p(1:2) - bb_sub_p(1, :)' + 1;
    [e_p_sub, cov_e] = alg.refine_ellipse_edges(sub_array_dx, ...
                                                sub_array_dy, ...
                                                e_p_sub, ...
                                                opts, ...
                                                double(mask_sub));

    % Get covariance of center
    cov_p = cov_e(1:2, 1:2);

    % Get ellipse in array coordinates.
    e_p = e_p_sub;
    e_p(1:2) = e_p_sub(1:2) + bb_sub_p(1, :)' - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(e_p(1:2)', bb_sub_p)
        e_p(:) = nan;
        cov_p(:) = nan;
        return
    end
end

function [e_p, cov_p] = dot(p_p_init, boundary_p_centered, array, array_dx, array_dy, opts)
    % Initialize ellipse by using dualconic method
    e_p = dualconic(p_p_init, boundary_p_centered, array, array_dx, array_dy, opts);
    cov_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array);

    % Get bounding box and mask of sub arrays
    [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_centered + e_p(1:2)');

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        e_p(:) = nan;
        cov_p(:) = nan;
        return
    end

    % Get sub array
    sub_array = alg.get_sub_array_bb(array, bb_sub_p);

    % Get refined ellipse; note that coordinates will be WRT sub_array.
    e_p_sub = e_p;
    e_p_sub(1:2) = e_p(1:2) - bb_sub_p(1, :)' + 1;
    [e_p_sub, cov_e] = alg.refine_ellipse_dot(sub_array, ...
                                              e_p_sub, ...
                                              opts, ...
                                              double(mask_sub));

    % Get covariance of center
    cov_p = cov_e(1:2, 1:2);

    % Get ellipse in array coordinates.
    e_p = e_p_sub;
    e_p(1:2) = e_p_sub(1:2) + bb_sub_p(1, :)' - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(e_p(1:2)', bb_sub_p)
        e_p(:) = nan;
        cov_p(:) = nan;
        return
    end
end
