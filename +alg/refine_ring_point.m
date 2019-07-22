function [p_p, cov_p, debug] = refine_ring_point(p_p_init, boundary_p, array, array_dx, array_dy, opts)
    % Performs refinement of a ring target on an array.
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
    %   p_p - array; 1x2 optimized ring pixel point
    %   cov_p - array; 2x2 covariance matrix of ring pixel point
    %   debug - struct;

    % Run optimization ---------------------------------------------------%

    switch opts.target_optimization
        case 'opencv'
            f_refine_target_point = @opencv;
        otherwise
            error(['Unsupported target optimization: "' opts.target_optimization '" for ring target.']);
    end

    % Get point
    [p_p, cov_p] = f_refine_target_point(p_p_init, ...
                                         boundary_p - p_p_init, ... % Center boundary
                                         array, ...
                                         array_dx, ...
                                         array_dy, ...
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

function [p_p, cov_p] = opencv(p_p_init, boundary_p_centered, array, array_dx, array_dy, opts)
    % Initialize point and covariance
    p_p = p_p_init;
    cov_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array);

    % Get initial bounding box
    bb_sub_p_init = calc_bb_and_mask(boundary_p_centered + p_p_init);

    % Perform iterations until convergence
    for it = 1:opts.refine_ring_opencv_it_cutoff
        % Cache previous point
        p_p_prev = p_p;

        % Get bounding box and mask of sub array
        [bb_sub_p, mask_sub] = calc_bb_and_mask(boundary_p_centered + p_p);

        % Check bounds
        if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
            p_p(:) = nan;
            cov_p(:) = nan;
            return
        end

        % Get sub arrays
        sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
        sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

        % Fit ring using opencv method; note that coordinates will be
        % WRT sub_array.
        [p_p_sub, cov_p] = alg.refine_ring_opencv(sub_array_dx, ...
                                                  sub_array_dy, ...
                                                  double(mask_sub));

        % Get point in array coordinates.
        p_p = p_p_sub + bb_sub_p(1, :) - 1;

        % Make sure point did not go outside of original bounding box
        if ~alg.is_p_in_bb(p_p, bb_sub_p_init)
            p_p(:) = nan;
            cov_p(:) = nan;
            return
        end

        % Exit if change in distance is small
        diff_norm = norm(p_p_prev - p_p);
        if diff_norm < opts.refine_ring_opencv_norm_cutoff
            break
        end
    end
end
