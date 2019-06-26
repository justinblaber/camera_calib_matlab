function blobs = blob_detect_LoG(array, opts)
    % Performs blob detection on input array; this returns dark blobs.
    %
    % Inputs:
    %   array - array; MxN array
    %   opts - struct;
    %       .blob_detect_LoG_r_range1 - scalar; starting point for blob
    %           radius search.
    %       .blob_detect_LoG_r_range2 - scalar; end point for blob radius
    %           search.
    %       .blob_detect_LoG_step - scalar; increment for blob radius
    %           search.
    %       .blob_detect_LoG_num_cutoff - int; max number of blobs found.
    %       .blob_detect_LoG_val_cutoff - scalar; cutoff for blob LoG
    %           value.
    %       .blob_detect_LoG_interp - string; interpolation used for
    %           LoG resampling.
    %       .blob_detect_LoG_eccentricity_cutoff - scalar; cutoff for
    %           eccentricity of blob.
    %       .blob_detect_LoG_lambda - scalar; damping factor added to
    %           hession
    %       .blob_detect_LoG_maxima_it_cutoff - int; max number of
    %           iterations performed for refinement of maxima parameters.
    %       .blob_detect_LoG_maxima_norm_cutoff - scalar; cutoff for norm
    %           of difference of parameter vector for refinement of maxima
    %           parameters.
    %       .blob_detect_LoG_centroid_it_cutoff - int; max number of
    %           iterations performed for refinement of centroid.
    %       .blob_detect_LoG_centroid_norm_cutoff - scalar; cutoff for norm
    %           of difference of parameter vector for refinement of
    %           centroid.
    %       .blob_detect_LoG_d_cluster - scalar; clusters blobs within d
    %           distance.
    %       .blob_detect_LoG_r1_cluster - scalar; clusters blobs within r1
    %           distance.
    %       .blob_detect_LoG_r2_cluster - scalar; clusters blobs within r2
    %           distance.
    %
    % Outputs:
    %   blobs - array; Px5 array containing:
    %       blobs(i, 1) = h; x component of center of blob
    %       blobs(i, 2) = k; y component of center of blob
    %       blobs(i, 3) = a; major axis length
    %       blobs(i, 4) = b; minor axis length
    %       blobs(i, 5) = alpha; rotation of major axis

    % Get bounding box of array
    bb_array = alg.bb_array(array);

    % Normalize array
    array = alg.normalize_array(array, 'min-max');

    % Precompute gradients
    array_dx = alg.grad_array(array, 'x');
    array_dy = alg.grad_array(array, 'y');

    % Get scale normalized LoG stack -------------------------------------%

    % Get number of scales
    r_range1 = opts.blob_detect_LoG_r_range1;
    r_range2 = opts.blob_detect_LoG_r_range2;
    step = opts.blob_detect_LoG_step;
    num_scales = ceil((r_range2-r_range1)/step) + 1;

    % Create scale normalized LoG stack
    stack_LoG = zeros([size(array) num_scales]);
    for i = 1:num_scales
        r = (i-1)*step + r_range1;                                        % radius
        sigma = r/sqrt(2);                                                % standard deviation
        kernel_LoG = sigma^2 * fspecial('log', 2*ceil(4*sigma)+1, sigma); % scale normalized LoG kernel

        % Apply LoG filter
        stack_LoG(:, :, i) = imfilter(array, kernel_LoG, 'same', 'replicate');
    end

    % Get interpolator
    I_stack_LoG = griddedInterpolant({1:size(stack_LoG, 1), 1:size(stack_LoG, 2), 1:size(stack_LoG, 3)}, ...
                                      stack_LoG, ...
                                      opts.blob_detect_LoG_interp, ...
                                      'none');

    % Get initial maxima of scale normalized LoG stack -------------------%

    % Assign, to every voxel, the maximum of its neighbors. Then, see if
    % voxel value is greater than this value; if true, it's a local maxima.
    kernel = true(3, 3, 3);
    kernel(2, 2, 2) = false;
    maxima = stack_LoG > imdilate(stack_LoG, kernel);

    % Clear out edge values
    maxima(  1,   :,   :) = false;
    maxima(end,   :,   :) = false;
    maxima(  :,   1,   :) = false;
    maxima(  :, end,   :) = false;
    maxima(  :,   :,   1) = false;
    maxima(  :,   :, end) = false;

    % Get initial coordinates of maxima
    idx_maxima_init = find(maxima);
    [y_maxima_init, x_maxima_init, idx_r_maxima_init] = ind2sub(size(maxima), idx_maxima_init);

    % Get most powerful blob responses specified by num_cutoff and
    % LoG_cutoff
    maxima_vals = stack_LoG(idx_maxima_init);
    [~, idx_maxima_sorted] = sort(maxima_vals, 'descend');
    idx_maxima_sorted = idx_maxima_sorted(1:min(opts.blob_detect_LoG_num_cutoff, end));
    idx_maxima_sorted = idx_maxima_sorted(maxima_vals(idx_maxima_sorted) > opts.blob_detect_LoG_val_cutoff);
    x_maxima_init = x_maxima_init(idx_maxima_sorted);
    y_maxima_init = y_maxima_init(idx_maxima_sorted);
    idx_r_maxima_init = idx_r_maxima_init(idx_maxima_sorted);

    % Initialize blobs ---------------------------------------------------%

    blobs = zeros(0, 5);

    % Iterate
    for i = 1:numel(x_maxima_init)
        % Get sub-pixel maxima values with gauss newton method -----------%

        % Grab initial values and then optimize
        params_maxima = [x_maxima_init(i); y_maxima_init(i); idx_r_maxima_init(i)];
        params_maxima = refine_LoG_maxima_params(params_maxima, ...
                                                 I_stack_LoG, ...
                                                 'full', ...
                                                 opts);

        % Make sure optimization was successful
        if any(~isfinite(params_maxima))
            continue
        end

        % Get optimized values
        x = params_maxima(1);
        y = params_maxima(2);
        r = (params_maxima(3)-1)*step + r_range1;

        % Get sub array containing blob ----------------------------------%

        % Get bounding box
        hw = ceil(4*r);
        bb_sub_array = [round(x)-hw round(y)-hw;
                        round(x)+hw round(y)+hw];

        % Make sure sub array is in bounds
        if ~alg.is_bb_in_bb(bb_sub_array, bb_array)
            continue
        end

        % Grab sub arrays
        sub_array = alg.get_sub_array_bb(array, bb_sub_array);
        sub_array_dx = alg.get_sub_array_bb(array_dx, bb_sub_array);
        sub_array_dy = alg.get_sub_array_bb(array_dy, bb_sub_array);

        % Get sub_array coordinates
        [y_sub_array, x_sub_array] = alg.ndgrid_bb(bb_sub_array);

        % Initialize ellipse ---------------------------------------------%

        e = [x y r r 0];

        % Refine ellipse with second moment matrix -----------------------%

        e = second_moment_ellipse(sub_array_dx, ...
                                  sub_array_dy, ...
                                  x_sub_array, ...
                                  y_sub_array, ...
                                  e, ...
                                  r);
        if any(~isfinite(e)) || e(3)/e(4) > opts.blob_detect_LoG_eccentricity_cutoff
            continue
        end

        % Refine center position of blob using centroid refinement -------%

        % Use inverse of sub array for centroid refinement
        sub_array_c = alg.normalize_array(-sub_array, 'min-max');

        % Iterate
        x = e(1);
        y = e(2);
        for it = 1:opts.blob_detect_LoG_centroid_it_cutoff
            % Store previous
            x_prev = x;
            y_prev = y;

            % Get weights
            W = weight_array(e, x_sub_array, y_sub_array);

            % Get refined x and y position
            sub_array_c_W = sub_array_c.*W;
            sum_sub_array_c_W = sum(sub_array_c_W(:));
            x = sum(sum(sub_array_c_W.*x_sub_array))/sum_sub_array_c_W;
            y = sum(sum(sub_array_c_W.*y_sub_array))/sum_sub_array_c_W;

            % Update center position of ellipse
            e(1) = x;
            e(2) = y;

            % Exit if change in distance is small
            diff_norm = norm([x_prev-x y_prev-y]);
            if diff_norm < opts.blob_detect_LoG_centroid_norm_cutoff
                break
            end
        end

        % Refine scale ---------------------------------------------------%

        % Convert r into index
        idx_r = (r-r_range1)/step+1;

        % Grab initial values and then optimize
        params_maxima = [e(1); e(2); idx_r];
        params_maxima = refine_LoG_maxima_params(params_maxima, ...
                                                 I_stack_LoG, ...
                                                 'idx_r', ...
                                                 opts);

        % Make sure optimization was successful
        if any(~isfinite(params_maxima))
            continue
        end

        % Get new r
        r = (params_maxima(3)-1)*step + r_range1;

        % Refine ellipse with second moment matrix and updated scale -----%

        e = second_moment_ellipse(sub_array_dx, ...
                                  sub_array_dy, ...
                                  x_sub_array, ...
                                  y_sub_array, ...
                                  e, ...
                                  r);
        if any(~isfinite(e)) || e(3)/e(4) > opts.blob_detect_LoG_eccentricity_cutoff
            continue
        end

        % Re-check LoG value ---------------------------------------------%

        % Convert r into index
        idx_r = (r-r_range1)/step+1;

        % Get value
        LoG_val = I_stack_LoG(e(2), e(1), idx_r);
        if ~isfinite(LoG_val) || LoG_val <= opts.blob_detect_LoG_val_cutoff
            continue
        end

        % Store blob -----------------------------------------------------%

        % Do some very rudimentary clustering
        dist_d = sqrt((blobs(:, 1)-e(1)).^2 + (blobs(:, 2)-e(2)).^2);
        dist_r1 = abs(blobs(:, 3)-e(3));
        dist_r2 = abs(blobs(:, 4)-e(4));
        if all(dist_d > opts.blob_detect_LoG_d_cluster | ...
               dist_r1 > opts.blob_detect_LoG_r1_cluster | ...
               dist_r2 > opts.blob_detect_LoG_r2_cluster)
            blobs(end+1, :)= e; %#ok<AGROW>
        end
    end
end

function params = refine_LoG_maxima_params(params, I_stack_LoG, optimization_type, opts)
    % Determine which parameters to update based on type
    idx_update = false(size(params));
    switch optimization_type
        case 'idx_r'
            % Only update radius
            idx_update(3) = true;
        case 'full'
            % Update everything
            idx_update(1:end) = true;
        otherwise
            error(['Input type of: "' optimization_type '" was not recognized']);
    end

    % Initialize "box" to interpolate in order to compute finite difference
    % approximations to gradient/hessian
    [y_box, x_box, idx_r_box] = ndgrid(-1:1:1, -1:1:1, -1:1:1);

    % Iterate
    for it = 1:opts.blob_detect_LoG_maxima_it_cutoff
        % Get finite difference coordinates for interpolation
        x_fd = x_box + params(1);
        y_fd = y_box + params(2);
        idx_r_fd = idx_r_box + params(3);

        % Interpolate
        LoG_fd = reshape(I_stack_LoG(y_fd(:), x_fd(:), idx_r_fd(:)), 3, 3, 3);

        % Check to make sure all sampled points were in bounds and valid
        if any(~isfinite(LoG_fd))
            params(:) = NaN;
            break
        end

        % Get gradient:
        %   [dLoG/dx dLoG/dy dLoG/didx_r]
        grad = [(LoG_fd(2, 3, 2)-LoG_fd(2, 1, 2))/2;
                (LoG_fd(3, 2, 2)-LoG_fd(1, 2, 2))/2;
                (LoG_fd(2, 2, 3)-LoG_fd(2, 2, 1))/2];

        % Get hessian:
        %   [d^2LoG/dx^2        d^2LoG/(dx*dy)      d^2LoG/(dx*didx_r)
        %    d^2LoG/(dy*dx)     d^2LoG/dy^2         d^2LoG/(dy*didx_r)
        %    d^2LoG/(didx_r*dx) d^2LoG/(didx_r*dy)  d^2LoG/(didx_r^2)]
        hess = [LoG_fd(2, 3, 2)-2*LoG_fd(2, 2, 2)+LoG_fd(2, 1, 2),                           ((LoG_fd(3, 3, 2)-LoG_fd(1, 3, 2))/2-(LoG_fd(3, 1, 2)-LoG_fd(1, 1, 2))/2)/2, ((LoG_fd(2, 3, 3)-LoG_fd(2, 3, 1))/2-(LoG_fd(2, 1, 3)-LoG_fd(2, 1, 1))/2)/2;
                ((LoG_fd(3, 3, 2)-LoG_fd(1, 3, 2))/2-(LoG_fd(3, 1, 2)-LoG_fd(1, 1, 2))/2)/2, LoG_fd(3, 2, 2)-2*LoG_fd(2, 2, 2)+LoG_fd(1, 2, 2),                           ((LoG_fd(3, 2, 3)-LoG_fd(3, 2, 1))/2-(LoG_fd(1, 2, 3)-LoG_fd(1, 2, 1))/2)/2;
                ((LoG_fd(2, 3, 3)-LoG_fd(2, 3, 1))/2-(LoG_fd(2, 1, 3)-LoG_fd(2, 1, 1))/2)/2, ((LoG_fd(3, 2, 3)-LoG_fd(3, 2, 1))/2-(LoG_fd(1, 2, 3)-LoG_fd(1, 2, 1))/2)/2, LoG_fd(2, 2, 3)-2*LoG_fd(2, 2, 2)+LoG_fd(2, 2, 1)];

        % Get incremental parameters
        try
            delta_params = linsolve(-hess(idx_update, idx_update) + opts.blob_detect_LoG_lambda*eye(sum(idx_update)), ... % Negative hess => positive definite
                                    grad(idx_update), ...
                                    struct('POSDEF', true, 'SYM', true));
        catch
            params(:) = NaN;
            break
        end

        if any(delta_params > 1)
            % Limit maximum change to 1 since maxima should not move
            % too much
            delta_params = delta_params./max(delta_params);
        end

        % Update params
        params(idx_update) = params(idx_update) + delta_params;

        % Exit if change in distance is small
        diff_norm = norm(delta_params);
        if diff_norm < opts.blob_detect_LoG_maxima_norm_cutoff
            break
        end
    end
end

function W = weight_array(e, xs, ys)
    [cov, p] = alg.ellipse2cov(e);
    W = alg.safe_mvnpdf([xs(:) ys(:)], p, cov);
    W = reshape(W, size(xs));
end

function e = second_moment_ellipse(array_dx, array_dy, xs, ys, e, r)
    % Get weight matrix
    W = weight_array(e, xs, ys);

    % Get second moment matrix
    M = zeros(2);
    M(1, 1) = sum(W(:).*array_dx(:).^2);
    M(1, 2) = sum(W(:).*array_dx(:).*array_dy(:));
    M(2, 2) = sum(W(:).*array_dy(:).^2);
    M(2, 1) = M(1, 2);

    % Get shape of ellipse from second moment matrix
    e = alg.cov2ellipse(alg.safe_inv(M), e(1:2)');

    % Constrain so:
    %   minor axis + major axis = 2*radius
    sf = 2*r/(e(3)+e(4));
    e(3) = sf*e(3);
    e(4) = sf*e(4);
end
