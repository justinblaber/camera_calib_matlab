function [p_p, cov_p, debug] = refine_checker_point(p_p_init, boundary_p, array, array_dx, array_dy, opts)
    % Performs refinement of a checker target on an array.
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
    %       .refine_checker_min_hw - int; minimum half window for checker
    %           refinement sub array
    %       .refine_checker_max_hw - int; maximum half window for checker
    %           refinement sub array
    %       .verbosity - int; level of verbosity
    %       .* - options for f_refine_target_point() function
    %
    % Outputs:
    %   p_p - array; 1x2 optimized checker pixel point
    %   cov_p - array; 2x2 covariance matrix of checker pixel point
    %   debug - struct;

    % Compute half window ------------------------------------------------%

    % Form lines from boundary points
    l_ps = cell(size(boundary_p, 1), 1);
    for i = 1:size(boundary_p, 1)-1
        l_ps{i} = alg.points2line(boundary_p(i, :), boundary_p(i+1, :));
    end
    l_ps{end} = alg.points2line(boundary_p(end, :), boundary_p(1, :));

    % Get shortest distance from lines to p_p_init
    d_ps = Inf(numel(l_ps), 1);
    for i = 1:numel(l_ps)
        d_ps(i) = alg.point_line_distance(p_p_init, l_ps{i});
    end

    % Get minimum distance
    d_p_min = min(d_ps);

    % Get half width of circumscribed square, which should help prevent
    % neighboring checkers from being inside the square.
    hw_p = floor(d_p_min/sqrt(2));
    if hw_p < opts.refine_checker_min_hw
        util.verbose_warning(['Minimum half width set, check to make sure ' ...
                              'checkers in this image are not too small.'], ...
                              1, ...
                              opts);
        hw_p = opts.refine_checker_min_hw;
    elseif hw_p > opts.refine_checker_max_hw
        hw_p = opts.refine_checker_max_hw;
    end

    % Run optimization ---------------------------------------------------%

    switch opts.target_optimization
        case 'opencv'
            f_refine_target_point = @opencv;
        case 'edges'
            f_refine_target_point = @edges;
        otherwise
            error(['Unsupported target optimization: "' opts.target_optimization '" for checker target.']);
    end

    % Get point
    [p_p, cov_p] = f_refine_target_point(p_p_init, ...
                                         hw_p, ...
                                         array, ...
                                         array_dx, ...
                                         array_dy, ...
                                         opts);

    % Set debugging output
    debug.hw_p = hw_p;
end

function W = weight_array(p, sigma, xs, ys)
    cov = [sigma^2 0; ...
           0       sigma^2];
    W = alg.safe_mvnpdf([xs(:) ys(:)], p, cov);
    W = reshape(W, size(xs));
end

function [p_p, cov_p] = opencv(p_p_init, hw_p, array, array_dx, array_dy, opts)
    % Initialize point and covariance
    p_p = p_p_init;
    cov_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array);

    % Perform iterations until convergence
    for it = 1:opts.refine_checker_opencv_it_cutoff
        % Cache previous point
        p_p_prev = p_p;

        % Get bounding box of sub array
        p_p_rounded = round(p_p);
        bb_sub_p = [p_p_rounded(1)-hw_p p_p_rounded(2)-hw_p;
                    p_p_rounded(1)+hw_p p_p_rounded(2)+hw_p];

        % Check bounds
        if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
            p_p(:) = nan;
            cov_p(:) = nan;
            return
        end

        % Get sub arrays
        sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
        sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

        % Get weights
        [y_sub_ps, x_sub_ps] = alg.ndgrid_bb(bb_sub_p);
        W = weight_array(p_p, hw_p/2, x_sub_ps, y_sub_ps);

        % Get refined point; note that coordinates will be WRT sub array.
        [p_p_sub, cov_p] = alg.refine_checker_opencv(sub_array_dx, sub_array_dy, W);

        % Get point in array coordinates.
        p_p = p_p_sub + bb_sub_p(1, :) - 1;

        % Make sure point did not go outside of original bounding box
        if any(abs(p_p - p_p_init) > hw_p)
            p_p(:) = nan;
            cov_p(:) = nan;
            return
        end

        % Exit if change in distance is small
        diff_norm = norm(p_p_prev - p_p);
        if diff_norm < opts.refine_checker_opencv_norm_cutoff
            break
        end
    end
end

function [p_p, cov_p] = edges(p_p_init, hw_p, array, array_dx, array_dy, opts)
    % Initialize point by using opencv method
    p_p = opencv(p_p_init, hw_p, array, array_dx, array_dy, opts);
    cov_p = nan(2, 2);

    % Get bb of array
    bb_array_p = alg.bb_array(array);

    % Get bounding box of sub array
    p_p_rounded = round(p_p);
    bb_sub_p = [p_p_rounded(1)-hw_p p_p_rounded(2)-hw_p;
                p_p_rounded(1)+hw_p p_p_rounded(2)+hw_p];

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        p_p(:) = nan;
        cov_p(:) = nan;
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

    % Get weights
    [y_sub_ps, x_sub_ps] = alg.ndgrid_bb(bb_sub_p);
    W = weight_array(p_p, hw_p/2, x_sub_ps, y_sub_ps);

    % Get initial line estimates using two most dominant gradient angles in
    % sub arrays
    angles = alg.dominant_grad_angles(sub_array_dx, ...
                                      sub_array_dy, ...
                                      2, ...
                                      opts, ...
                                      W);
    angles = angles + pi/2; % Line is perpendicular to gradient
    l1_p_sub = alg.pointslope2line(p_p - bb_sub_p(1, :) + 1, tan(angles(1)));
    l2_p_sub = alg.pointslope2line(p_p - bb_sub_p(1, :) + 1, tan(angles(2)));

    % Get refined point; note that coordinates will be WRT sub_array.
    [p_p_sub, cov_p] = alg.refine_checker_edges(sub_array_dx, ...
                                                sub_array_dy, ...
                                                l1_p_sub, ...
                                                l2_p_sub, ...
                                                opts, ...
                                                W);

    % Get point in array coordinates.
    p_p = p_p_sub + bb_sub_p(1, :) - 1;

    % Make sure point did not go outside of original bounding box
    if any(abs(p_p - p_p_init) > hw_p)
        p_p(:) = nan;
        cov_p(:) = nan;
        return
    end
end
