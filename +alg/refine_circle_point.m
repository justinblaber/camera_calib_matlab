function [p_cb_p, cov_cb_p, debug] = refine_circle_point(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts)
    % Performs refinement of a circle target on a calibration board image
    % array.
    %
    % Inputs:
    %   p_cb_p_init - array; 1x2 initial guess for target pixel point
    %   boundary_p_center - array; boundary points around target point in
    %       pixel coordinates
    %   array_cb - array; MxN calibration board array
    %   array_cb_dx - array; MxN gradient array in x direction
    %   array_cb_dy - array; MxN gradient array in y direction
    %   opts - struct;
    %       .target_optimization - string; optimization type for
    %           calibration target
    %       .* - options for f_refine_target_point() function
    %
    % Outputs:
    %   p_cb_p - array; 1x2 optimized circle pixel point
    %   cov_cb_p - array; 2x2 covariance matrix of circle pixel point
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
            error(['Unsupported target optimization: "' opts.target_optimization '" for circle target.']);
    end

    % Get ellipse
    [e_cb_p, cov_cb_p] = f_refine_target_point(p_cb_p_init, ...
                                               boundary_p_center, ...
                                               array_cb, ...
                                               array_cb_dx, ...
                                               array_cb_dy, ...
                                               opts);

    % Parse point
    p_cb_p = e_cb_p(1:2)';

    % Set debugging output
    debug.e_cb_p = e_cb_p;
    debug.boundary_p_center = boundary_p_center;
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

function [e_cb_p, cov_cb_p] = dualconic(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts) %#ok<INUSD>
    % Initialize ellipse and covariance
    e_cb_p = nan(5, 1);
    cov_cb_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array_cb);

    % Get bounding box and mask of sub array
    [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_center + p_cb_p_init);

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        e_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_cb_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_cb_dy, bb_sub_p);

    % Apply mask
    sub_array_dx(~mask_sub) = 0;
    sub_array_dy(~mask_sub) = 0;

    % Fit ellipse using dual conic method; note that coordinates will be
    % WRT sub_array.
    e_cb_p_sub = alg.refine_ellipse_dualconic(sub_array_dx, ...
                                              sub_array_dy, ...
                                              double(mask_sub));

    % TODO: update refine_ellipse_dualconic to return covariance of ellipse
    % center; for now just return an identity matrix.
    cov_cb_p = eye(2);

    % Get ellipse in array coordinates.
    e_cb_p = e_cb_p_sub;
    e_cb_p(1:2) = e_cb_p(1:2) + bb_sub_p(1, :)' - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(e_cb_p(1:2)', bb_sub_p)
        e_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end
end

function [e_cb_p, cov_cb_p] = edges(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts)
    % Initialize ellipse by using dualconic method
    e_cb_p = dualconic(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts);
    cov_cb_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array_cb);

    % Get bounding box and mask of sub arrays
    [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_center + e_cb_p(1:2)');

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        e_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_cb_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_cb_dy, bb_sub_p);

    % Apply mask
    sub_array_dx(~mask_sub) = 0;
    sub_array_dy(~mask_sub) = 0;

    % Get refined ellipse; note that coordinates will be WRT sub_array.
    e_cb_p_sub = e_cb_p;
    e_cb_p_sub(1:2) = e_cb_p(1:2) - bb_sub_p(1, :)' + 1;
    [e_cb_p_sub, cov_cb_e] = alg.refine_ellipse_edges(sub_array_dx, ...
                                                      sub_array_dy, ...
                                                      e_cb_p_sub, ...
                                                      opts, ...
                                                      double(mask_sub));

    % Get covariance of center
    cov_cb_p = cov_cb_e(1:2, 1:2);

    % Get ellipse in array coordinates.
    e_cb_p = e_cb_p_sub;
    e_cb_p(1:2) = e_cb_p_sub(1:2) + bb_sub_p(1, :)' - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(e_cb_p(1:2)', bb_sub_p)
        e_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end
end

function [e_cb_p, cov_cb_p] = dot(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts)
    % Initialize ellipse by using dualconic method
    e_cb_p = dualconic(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts);
    cov_cb_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array_cb);

    % Get bounding box and mask of sub arrays
    [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_center + e_cb_p(1:2)');

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        e_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end

    % Get sub array
    sub_array = alg.get_sub_array_bb(array_cb, bb_sub_p);

    % Apply mask
    sub_array(~mask_sub) = 0;

    % Get refined ellipse; note that coordinates will be WRT sub_array.
    e_cb_p_sub = e_cb_p;
    e_cb_p_sub(1:2) = e_cb_p(1:2) - bb_sub_p(1, :)' + 1;
    [e_cb_p_sub, cov_cb_e] = alg.refine_ellipse_dot(sub_array, ...
                                                    e_cb_p_sub, ...
                                                    opts, ...
                                                    double(mask_sub));

    % Get covariance of center
    cov_cb_p = cov_cb_e(1:2, 1:2);

    % Get ellipse in array coordinates.
    e_cb_p = e_cb_p_sub;
    e_cb_p(1:2) = e_cb_p_sub(1:2) + bb_sub_p(1, :)' - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(e_cb_p(1:2)', bb_sub_p)
        e_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end
end