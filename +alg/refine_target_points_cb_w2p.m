function [p_cb_ps, cov_cb_ps, idx_valid, debugs] = refine_target_points_cb_w2p(p_cb_ws, boundary_ws, array_cb, f_p_cb_w2p_cb_p, f_p_w2p_p, opts)
    % Performs refinement of target points on a calibration board image
    % array.
    %
    % Inputs:
    %   p_cb_ws - array; Nx2 array of calibration board world points
    %   boundary_ws - cell; Nx1 cell array of boundary points around target
    %       point in world coordinates
    %   array_cb - array; MxN calibration board array
    %   f_p_cb_w2p_cb_p - function handle; function which transforms
    %       calibration world points to calibration pixel points
    %   f_p_w2p_p - function handle; function which transforms points from
    %       world coordinates to pixel coordinates
    %   opts - struct;
    %       .target - string; type of calibration target
    %       .target_optimization - string; optimization type for
    %           calibration target
    %       .* - rest of options are for f_refine_target_point()
    %
    % Outputs:
    %   p_cb_ps - array; Px2 array of optimized target pixel points
    %   cov_cb_ps - cell array; Px1 cell array of covariance matrices of
    %       target pixel points
    %   idx_valid - array; Px1 logical array of "valid" target points
    %   debugs - cell array;

    % Get target refinement function
    switch opts.target
        case 'checker'
            f_refine_target_point = @alg.refine_checker_point;
        case 'circle'
            f_refine_target_point = @alg.refine_circle_point;
        otherwise
            error(['Unknown calibration target: "' opts.target '"']);
    end

    % Get array bounding box
    bb_array = alg.bb_array(array_cb);

    % Precompute array gradients
    array_cb_dx = alg.grad_array(array_cb, 'x');
    array_cb_dy = alg.grad_array(array_cb, 'y');

    % Iterate over points and refine them; also keep track of which points
    % are "valid"
    p_cb_ps = zeros(size(p_cb_ws));
    cov_cb_ps = cell(size(p_cb_ws, 1), 1);
    idx_valid = false(size(p_cb_ws, 1), 1);
    debugs = cell(size(p_cb_ws, 1), 1);
    for i = 1:size(p_cb_ws, 1)
        % Get initial guess for calibration board pixel point; use
        % f_p_cb_w2p_cb_p() transform.
        p_cb_p_init = f_p_cb_w2p_cb_p(p_cb_ws(i, :));

        % Get boundary in pixel coordinates centered around point; use
        % f_p_w2p_p() transform.
        boundary_p_center = f_p_w2p_p(boundary_ws{i}) - p_cb_p_init;

        % Refine target point
        [p_cb_p, cov_cb_p, debug] = f_refine_target_point(p_cb_p_init, ...
                                                          boundary_p_center, ...
                                                          array_cb, ...
                                                          array_cb_dx, ...
                                                          array_cb_dy, ...
                                                          opts);

        % Make sure:
        % 1) p_cb_p is in array bounds
        % 2) cov_cb_p is positive definite. It must be positive definite
        %   because lscov() for sparse matrices requires covariance matrix
        %   to be positive definite.
        if alg.is_p_in_bb(p_cb_p, bb_array) && alg.is_pos_def(cov_cb_p)
            p_cb_ps(i, :) = p_cb_p;
            cov_cb_ps{i} = cov_cb_p;
            idx_valid(i) = true;
            debugs{i} = debug;
        end
    end
end
