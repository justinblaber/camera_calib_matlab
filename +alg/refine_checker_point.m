function [p_cb_p, cov_cb_p] = refine_checker_point(p_cb_p_init, boundary_p_center, array_cb, array_cb_dx, array_cb_dy, opts)
    % Performs refinement of center of checker target on a calibration
    % board image array.
    %
    % Inputs:
    %
    %
    % Outputs:
    %   p_cb_p - array; 1x2 optimized checker pixel point
    %   cov_cb_p - array; 2x2 covariance matrix of checker pixel point

    % First step is to compute half window -------------------------------%

    % Form lines
    for i = 1:size(boundary_p_center, 1)-1
        l1_ps{i} = alg.points2line(boundary_p_center(i, :), boundary_p_center(i+1, :)); %#ok<AGROW>
    end
    l1_ps{end+1} = alg.points2line(boundary_p_center(end, :), boundary_p_center(i, :));

    % Get shortest distance from lines to center (zero)
    for i = 1:numel(l1_ps)
        d_ps(i) = alg.point_line_distance([0 0], l1_ps{i}); %#ok<AGROW>
    end

    % Get minimum distance
    d_p_min = min(d_ps);

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

    % Determine which optimization to run
    switch opts.target_optimization
        case 'opencv'
            f_refine_target_point = @opencv;
        case 'edges'
            f_refine_target_point = @edges;
        otherwise
            error(['Unknown target optimization: "' opts.target_optimization '" for target type: "' opts.target_type '"']);
    end

    [p_cb_p, cov_cb_p] = f_refine_target_point(p_cb_p_init, ...
                                               array_cb, ...
                                               array_cb_dx, ...
                                               array_cb_dy, ...
                                               hw_p, ...
                                               opts);
end

function [p_cb_p, cov_cb_p] = opencv(p_cb_p_init, array_cb, array_dx, array_dy, hw_p, opts) %#ok<INUSL>
    % Get bb of array
    bb_array_p = alg.bb_array(array_dx);

    % Initialize point
    p_cb_p = p_cb_p_init;

    % Perform iterations until convergence
    for it = 1:opts.refine_checker_opencv_it_cutoff
        % Get bounding box of sub array
        p_cb_p_rounded = round(p_cb_p);
        bb_sub_p = [p_cb_p_rounded(1)-hw_p p_cb_p_rounded(2)-hw_p;
                    p_cb_p_rounded(1)+hw_p p_cb_p_rounded(2)+hw_p];

        % Check bounds
        if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
            % An output with nans indicates that this process failed
            p_cb_p = nan(1, 2);
            return
        end

        % Get sub arrays
        sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
        sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

        % Cache previous point
        p_cb_p_prev = p_cb_p;

        % Get refined point; note that coordinates will be WRT sub array.
        p_cb_p_sub = alg.refine_checker_opencv(sub_array_dx, ...
                                               sub_array_dy, ...
                                               p_cb_p - bb_sub_p(1, :) + 1);

        % Get point in array coordinates.
        p_cb_p = p_cb_p_sub + bb_sub_p(1, :) - 1;

        % Make sure point did not go outside of original bounding box
        if any(abs(p_cb_p - p_cb_p_init) > hw_p)
            p_cb_p = nan(1, 2);
            return
        end

        % Exit if change in distance is small
        diff_norm = norm(p_cb_p_prev - p_cb_p);
        if diff_norm < opts.refine_checker_opencv_norm_cutoff
            break
        end
    end
end

function [p_cb_p, cov_cb_p, bb_sub_p] = edges(p_cb_p_init, p_cb_w, f_p_cb_w2p_cb_p, array_dx, array_dy, hw_p, opts)
    % Get bb of array
    bb_array_p = alg.bb_array(array_dx);

    % Initialize point
    p_cb_p = p_cb_p_init;

    % Get bounding box of sub array
    p_cb_p_rounded = round(p_cb_p);
    bb_sub_p = [p_cb_p_rounded(1)-hw_p p_cb_p_rounded(2)-hw_p;
                p_cb_p_rounded(1)+hw_p p_cb_p_rounded(2)+hw_p];

    % Check bounds
    if ~alg.is_bb_in_bb(bb_sub_p, bb_array_p)
        % An output with nans indicates that this process failed
        p_cb_p = nan(1, 2);
        cov_cb_p = nan(2);
        return
    end

    % Get sub arrays
    sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_p);
    sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_p);

    % Get diamond around calibration board world point
    % Note:
    %         p2
    %         |
    %    p1 - p - p4
    %         |
    %         p3
    diamond_w = [p_cb_w(1)-opts.target_spacing p_cb_w(2);
                 p_cb_w(1) p_cb_w(2)-opts.target_spacing;
                 p_cb_w(1) p_cb_w(2)+opts.target_spacing;
                 p_cb_w(1)+opts.target_spacing p_cb_w(2)];

    % Apply xform to bring to pixel coordinates
    diamond_p = f_p_cb_w2p_cb_p(diamond_w);

    % Get points in sub array coordinates
    diamond_p_sub = diamond_p - bb_sub_p(1, :) + 1;

    % Get initial line estimates
    l1_p_sub = alg.pointslope2line(p_cb_p - bb_sub_p(1, :) + 1, ...
                                   (diamond_p_sub(4, 2)-diamond_p_sub(1, 2))/(diamond_p_sub(4, 1)-diamond_p_sub(1, 1)));
    l2_p_sub = alg.pointslope2line(p_cb_p - bb_sub_p(1, :) + 1, ...
                                   (diamond_p_sub(3, 2)-diamond_p_sub(2, 2))/(diamond_p_sub(3, 1)-diamond_p_sub(2, 1)));

    % Get refined point; note that coordinates will be WRT sub_array.
    [p_cb_p_sub, cov_cb_p] = alg.refine_checker_edges(sub_array_dx, ...
                                                      sub_array_dy, ...
                                                      l1_p_sub, ...
                                                      l2_p_sub, ...
                                                      opts);

    % Get point in array coordinates.
    p_cb_p = p_cb_p_sub + bb_sub_p(1, :) - 1;

    % Make sure p_cb_p did not go outside of original bounding box
    if any(abs(p_cb_p - p_cb_p_init) > hw_p)
        p_cb_p = nan(1, 2);
        cov_cb_p = nan(2);
        return
    end
end
