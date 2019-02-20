function [f_p_cb_w2p_cb_p, f_dp_cb_p_dh, f_homography_cb_w2p] = get_p_cb_w2p_cb_p(opts)
    % Returns function which transforms calibration board world points to
    % calibration board pixel points, its derivative with respect to
    % homography parameters, and the appropriate homography estimation
    % function using these points.
    %
    % Inputs:
    %   opts - struct;
    %       .calib_optimization - string; type of camera calibration
    %           optimization
    %       .target - string; camera calibration target
    %       .* - other options dependent on calibration optimization and
    %           target type.
    %
    % Outputs:
    %   f_p_cb_w2p_cb_p - function handle; function which transforms
    %       calibration board world points to calibration board pixel
    %       points
    %   f_dp_cb_p_dh - function handle; derivative of f_p_cb_w2p_cb_p with
    %       respect to homography parameters
    %   f_homography_cb_w2p - function_handle; calibration board homography
    %       estimation function using calibration board world points and
    %       calibration board pixel points

    % TODO: possibly make this a virtual method in a "calibration class".

    switch opts.calib_optimization
        case 'distortion_refinement'
            switch opts.target
                case 'checker'
                    f_p_cb_w2p_cb_p = @(p, H)alg.apply_homography_p2p(p, H);
                    f_dp_cb_p_dh = @(p, H)alg.dp_dh_p2p(p, H);
                    f_homography_cb_w2p = @alg.homography_p2p;
                case 'circle'
                    f_p_cb_w2p_cb_p = @(p, H)alg.apply_homography_c2e(p, H, opts.circle_radius);
                    f_dp_cb_p_dh = @(p, H)alg.dp_dh_c2e(p, H, opts.circle_radius);
                    f_homography_cb_w2p = @alg.homography_c2e;
                otherwise
                    error(['Invalid calibration target: "' opts.target '" for distortion refinement calibration.']);
            end
        case 'frontal_refinement'
            error('Frontal refinement has not been implemented yet.');
        otherwise
            error(['Unknown calibration optimization: "' opts.calib_optimization '"']);
    end
end
