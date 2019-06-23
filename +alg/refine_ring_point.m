function [p_cb_p, cov_cb_p, debug] = refine_ring_point(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts)
    % Performs refinement of a ring target on a calibration board image
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
    %   p_cb_p - array; 1x2 optimized ring pixel point
    %   cov_cb_p - array; 2x2 covariance matrix of ring pixel point
    %   debug - struct;

    % Run optimization ---------------------------------------------------%

    switch opts.target_optimization
        case 'opencv'
            f_refine_target_point = @opencv;
        otherwise
            error(['Unsupported target optimization: "' opts.target_optimization '" for ring target.']);
    end

    % Get point
    [p_cb_p, cov_cb_p] = f_refine_target_point(p_cb_p_init, ...
                                               boundary_p_center, ...
                                               array_cb, ...
                                               array_cb_dx, ...
                                               array_cb_dy, ...
                                               opts);

    % Set debugging output
    debug = struct();
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

function [p_cb_p, cov_cb_p] = opencv(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts) %#ok<INUSD>
    % Initialize point and covariance
    p_cb_p = p_cb_p_init;
    cov_cb_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array_cb);

    % Get bounding box and mask of sub array
    [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_center + p_cb_p_init);

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        p_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_cb_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_cb_dy, bb_sub_p);

    % Fit ring using opencv method; note that coordinates will be
    % WRT sub_array.
    [p_cb_p_sub, cov_cb_p] = alg.refine_ring_opencv(sub_array_dx, ...
                                                    sub_array_dy, ...
                                                    double(mask_sub));

    % Get point in array coordinates.
    p_cb_p = p_cb_p_sub + bb_sub_p(1, :) - 1;

    % Make sure point did not go outside of original bounding box
    if ~alg.is_p_in_bb(p_cb_p, bb_sub_p)
        p_cb_p(:) = nan;
        cov_cb_p(:) = nan;
        return
    end
end
