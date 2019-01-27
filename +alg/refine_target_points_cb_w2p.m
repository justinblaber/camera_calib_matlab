function [p_cb_ps, cov_cb_ps, idx_valid, debug] = refine_target_points_cb_w2p(array_cb, f_p_cb_w2p_cb_p, opts)
    % Performs refinement of center of target points on a calibration
    % board image array.
    %
    % Inputs:
    %
    % Outputs:
    %   p_cb_ps - array; Px2 array of optimized target pixel points
    %   cov_cb_ps - cell array; Px1 cell array of covariance matrices of
    %       target pixel points
    %   idx_valid - array; Px1 logical array of "valid" target points
    %   debug - cell array;

    % Get target refinement function
    switch opts.target
        case 'checker'
            f_refine_target_point = @alg.refine_checker_point;
        case 'circle'

        otherwise
            error(['Unknown calibration target: "' opts.target '"']);
    end

    % Get calibration board world points and boundaries
    p_cb_ws = opts.cb_class.get_p_cb_ws();
    boundary_ws = opts.cb_class.get_p_cb_w_boundaries();

    % Get array gradients
    array_cb_dx = alg.grad_array(array_cb, 'x');
    array_cb_dy = alg.grad_array(array_cb, 'y');

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

        % Refine target point
        [p_cb_p, cov_cb_p] = f_refine_target_point(p_cb_p_init, ...
                                                   boundary_p_center, ...
                                                   array_cb, ...
                                                   array_cb_dx, ...
                                                   array_cb_dy, ...
                                                   opts);

        % Make sure:
        % 1) p_cb_p_edges and cov_cb_p_edges are valid
        % 2) cov_cb_p_edges is positive definite. lscov() for sparse
        %   matrices requires covariance matrix to be positive definite. If
        %   the model is perfect, then covariance will be zero, but this is
        %   basically an impossibility with real data.
        % 3) points from opencv and edges estimation are close; if not,
        %   this means this checker is unreliable
        if all(isfinite(p_cb_p_edges)) && ...
           all(isfinite(cov_cb_p_edges(:))) && ...
           alg.is_pos_def(cov_cb_p_edges) && ...
           norm(p_cb_p_opencv - p_cb_p_edges) < opts.refine_checker_opencv_edges_diff_norm_cutoff
            p_cb_ps(i, :) = p_cb_p_edges;
            cov_cb_ps{i} = cov_cb_p_edges;
            idx_valid(i) = true;
            debug{i} = bb_sub_p_edges;
        end
    end
end
