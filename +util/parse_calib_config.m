function [calib_config, data] = parse_calib_config(data)
    % Parses calibration config from input data struct. Fields are
    % validated and fields which aren't present and aren't required are
    % given a default value.
    %
    % Inputs:
    %   data - struct; struct containing calibration config
    %
    % Outputs:
    %   calib_config - struct; calibration config
    %   data - struct; input data with calib config removed.

    % Calibration stuff --------------------------------------------------%

    % Calibration board target
    field_info        = struct('field', 'target'                                        , 'required', true , 'default', ''                             , 'validation_fun', @validate_target);
    field_info(end+1) = struct('field', 'target_optimization'                           , 'required', true , 'default', ''                             , 'validation_fun', @validate_target_optimization);
    
    % Calibration board geometry
    field_info(end+1) = struct('field', 'height_cb'                                     , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'width_cb'                                      , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'num_targets_height'                            , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar_int_or_nan);
    field_info(end+1) = struct('field', 'num_targets_width'                             , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar_int_or_nan);
    field_info(end+1) = struct('field', 'target_spacing'                                , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar_or_nan);
    field_info(end+1) = struct('field', 'first_on'                                      , 'required', false, 'default', nan                            , 'validation_fun', @validate_logical_scalar_or_nan);
    field_info(end+1) = struct('field', 'height_fp'                                     , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar_or_nan);
    field_info(end+1) = struct('field', 'width_fp'                                      , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar_or_nan);
    field_info(end+1) = struct('field', 'idx_target_removal'                            , 'required', false, 'default', []                             , 'validation_fun', @validate_pos_int);
    field_info(end+1) = struct('field', 'obj_cb_geom'                                   , 'required', true , 'default', []                             , 'validation_fun', @validate_obj_cb_geom);
    
    % Camera matrix parameterization
    field_info(end+1) = struct('field', 'A_parameterization'                            , 'required', false, 'default', 'single_focal'                 , 'validation_fun', @validate_A_parameterization);
        
    % Rotation matrix parameterization
    field_info(end+1) = struct('field', 'R_parameterization'                            , 'required', false, 'default', 'euler'                        , 'validation_fun', @validate_R_parameterization);
        
    % Distortion
    field_info(end+1) = struct('field', 'sym_p_p2p_p_d'                                 , 'required', false, 'default', 'distortion.heikkila97'        , 'validation_fun', @validate_sym_p_p2p_p_d);

    % Algorithms ---------------------------------------------------------%
    
    % Calibration optimization
    field_info(end+1) = struct('field', 'calib_optimization'                            , 'required', true , 'default', ''                             , 'validation_fun', @validate_calib_optimization);
    field_info(end+1) = struct('field', 'distortion_refinement_it_cutoff'               , 'required', false, 'default', 1                              , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'apply_covariance_optimization'                 , 'required', false, 'default', true                           , 'validation_fun', @validate_logical_scalar);
    
    % Homography computation
    field_info(end+1) = struct('field', 'homography_p2p_it_cutoff'                      , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'homography_p2p_norm_cutoff'                    , 'required', false, 'default', 1e-6                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'homography_c2e_it_cutoff'                      , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'homography_c2e_norm_cutoff'                    , 'required', false, 'default', 1e-6                           , 'validation_fun', @validate_pos_scalar);
    
    % Dominant grad angles
    field_info(end+1) = struct('field', 'dominant_grad_angles_num_bins'                 , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'dominant_grad_angles_space_peaks'              , 'required', false, 'default', 1                              , 'validation_fun', @validate_pos_scalar_int);

    % Checker refinement
    field_info(end+1) = struct('field', 'refine_checker_min_hw'                         , 'required', false, 'default', 5                              , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'refine_checker_max_hw'                         , 'required', false, 'default', 15                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'refine_checker_opencv_it_cutoff'               , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'refine_checker_opencv_norm_cutoff'             , 'required', false, 'default', 0.001                          , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'refine_checker_edges_it_cutoff'                , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'refine_checker_edges_norm_cutoff'              , 'required', false, 'default', 0.001                          , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'refine_checker_edges_h2_init'                  , 'required', false, 'default', 0.75                           , 'validation_fun', @validate_pos_scalar);

    % Ellipse refinement
    field_info(end+1) = struct('field', 'circle_radius'                                 , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar_or_nan);
    field_info(end+1) = struct('field', 'refine_ellipse_edges_it_cutoff'                , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'refine_ellipse_edges_norm_cutoff'              , 'required', false, 'default', 0.001                          , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'refine_ellipse_edges_h2_init'                  , 'required', false, 'default', 0.75                           , 'validation_fun', @validate_pos_scalar);

    % p_p_d2p_p
    field_info(end+1) = struct('field', 'p_p_d2p_p_it_cutoff'                           , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'p_p_d2p_p_norm_cutoff'                         , 'required', false, 'default', 1e-6                           , 'validation_fun', @validate_pos_scalar);

    % Undistort array
    field_info(end+1) = struct('field', 'undistort_array_interp'                        , 'required', false, 'default', 'spline'                       , 'validation_fun', @validate_interp);

    % Distort array
    field_info(end+1) = struct('field', 'distort_array_interp'                          , 'required', false, 'default', 'spline'                       , 'validation_fun', @validate_interp);

    % Single calibration
    field_info(end+1) = struct('field', 'refine_single_params_it_cutoff'                , 'required', false, 'default', 200                            , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'refine_single_params_norm_cutoff'              , 'required', false, 'default', 1e-6                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'refine_single_params_lambda_init'              , 'required', false, 'default', 0.01                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'refine_single_params_lambda_factor'            , 'required', false, 'default', 10                             , 'validation_fun', @validate_pos_scalar);

    % Stereo calibration
    field_info(end+1) = struct('field', 'refine_stereo_params_it_cutoff'                , 'required', false, 'default', 200                            , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'refine_stereo_params_norm_cutoff'              , 'required', false, 'default', 1e-6                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'refine_stereo_params_lambda_init'              , 'required', false, 'default', 0.01                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'refine_stereo_params_lambda_factor'            , 'required', false, 'default', 10                             , 'validation_fun', @validate_pos_scalar);

    % Blob detection
    field_info(end+1) = struct('field', 'blob_detect_r_range1'                          , 'required', false, 'default', 1                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_r_range2'                          , 'required', false, 'default', 15                             , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_step'                              , 'required', false, 'default', 0.5                            , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_num_cutoff'                        , 'required', false, 'default', 1000                           , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'blob_detect_LoG_cutoff'                        , 'required', false, 'default', 0.1                            , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_LoG_interp'                        , 'required', false, 'default', 'cubic'                        , 'validation_fun', @validate_interp);
    field_info(end+1) = struct('field', 'blob_detect_eccentricity_cutoff'               , 'required', false, 'default', 5                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_lambda'                            , 'required', false, 'default', 1e-2                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_maxima_it_cutoff'                  , 'required', false, 'default', 10                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'blob_detect_maxima_norm_cutoff'                , 'required', false, 'default', 1e-6                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_centroid_it_cutoff'                , 'required', false, 'default', 10                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'blob_detect_centroid_norm_cutoff'              , 'required', false, 'default', 0.1                            , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_d_cluster'                         , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_r1_cluster'                        , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'blob_detect_r2_cluster'                        , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar);

    % Four point detection
    field_info(end+1) = struct('field', 'fp_detector'                                   , 'required', false, 'default', 'LoG'                          , 'validation_fun', @validate_fp_detector);
    field_info(end+1) = struct('field', 'ellipse_detect_num_samples_theta'              , 'required', false, 'default', 100                            , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'ellipse_detect_interp'                         , 'required', false, 'default', 'cubic'                        , 'validation_fun', @validate_interp);
    field_info(end+1) = struct('field', 'ellipse_detect_sf_cost'                        , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'ellipse_detect_it_cutoff'                      , 'required', false, 'default', 100                            , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'ellipse_detect_norm_cutoff'                    , 'required', false, 'default', 1e-3                           , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'ellipse_detect_lambda_init'                    , 'required', false, 'default', 1                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'ellipse_detect_lambda_factor'                  , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'ellipse_detect_d_cluster'                      , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'ellipse_detect_r1_cluster'                     , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'ellipse_detect_r2_cluster'                     , 'required', false, 'default', 2                              , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'fp_detect_marker_templates_path'               , 'required', false, 'default', '+markers/marker_templates.txt', 'validation_fun', @validate_file);
    field_info(end+1) = struct('field', 'fp_detect_marker_config_path'                  , 'required', false, 'default', '+markers/marker.conf'         , 'validation_fun', @validate_file);
    field_info(end+1) = struct('field', 'fp_detect_num_cutoff'                          , 'required', false, 'default', 20                             , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'fp_detect_mse_cutoff'                          , 'required', false, 'default', 0.2                            , 'validation_fun', @validate_pos_scalar);
    field_info(end+1) = struct('field', 'fp_detect_padding_radial'                      , 'required', false, 'default', 5                              , 'validation_fun', @validate_pos_scalar_int);
    field_info(end+1) = struct('field', 'fp_detect_array_min_size'                      , 'required', false, 'default', 400                            , 'validation_fun', @validate_pos_scalar_int_or_nan);
    
    % Verbosity
    field_info(end+1) = struct('field', 'verbosity'                                     , 'required', false, 'default', 3                              , 'validation_fun', @validate_int_scalar);

    % Plotting info
    field_info(end+1) = struct('field', 'units'                                         , 'required', false, 'default', 'N/A'                          , 'validation_fun', @validate_string);
    field_info(end+1) = struct('field', 'camera_size'                                   , 'required', false, 'default', nan                            , 'validation_fun', @validate_pos_scalar_or_nan);

    % Create calib_config ------------------------------------------------%

    % Validate all inputs
    fields_data = fields(data);
    for i = 1:numel(field_info)
        % Get field
        field = field_info(i).field;

        % Check if field exists in data
        if any(strcmp(field, fields_data))
            % Field exists; get the value and remove it
            [param, data] = util.read_and_remove(data, field);

            % Set value
            calib_config.(field) = param;
        else
            % Field doesn't exist; check if it's required or not
            if field_info(i).required
                error(['Required field: "' field '" was ' ...
                       'not found.']);
            else
                % This is an optional field; set the default value since it
                % was not set
                calib_config.(field) = field_info(i).default;
            end
        end

        % Validate field if a validation function was set
        if ~isempty(field_info(i).validation_fun)
            calib_config = field_info(i).validation_fun(calib_config, field);
        end
    end
end

% Make all validate_* functions take calib_config and the field, and return
% the calib_config. This makes things easier.

function calib_config = validate_target(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if ~any(strcmp(param, {'checker', 'circle'}))
        field_class_error(field, param, 'calibration target');
    end
end

function calib_config = validate_target_optimization(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if ~any(strcmp(param, {'opencv', 'dualconic', 'edges'}))
        field_class_error(field, param, 'calibration target optimization');
    end
end

function calib_config = validate_calib_optimization(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if ~any(strcmp(param, {'distortion_refinement'}))
        field_class_error(field, param, 'calibration optimization');
    end
end

function calib_config = validate_obj_cb_geom(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    % Must start with "class."
    if ~startsWith(param, 'class.')
        field_class_error(field, param, 'class in +class directory');
    end

    % Evaluate
    [~, obj_cb_geom] = evalc([param '(calib_config)']);

    % TODO: validate

    % Assign value
    calib_config.(field) = obj_cb_geom;
end

function calib_config = validate_fp_detector(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if ~isempty(param) && ~any(strcmp(param, {'LoG'}))
        field_class_error(field, param, 'four point detector');
    end
end

function calib_config = validate_string(calib_config, field)
    param = calib_config.(field);

    if ~ischar(param)
        field_class_error(field, param, 'string');
    end
end

function calib_config = validate_scalar(calib_config, field)
    param = calib_config.(field);

    if ~isscalar(param)
        field_class_error(field, param, 'scalar');
    end
end

function calib_config = validate_int(calib_config, field)
    param = calib_config.(field);

    if ~all(alg.is_int(param))
        field_class_error(field, param, 'integer array');
    end
end

function calib_config = validate_pos(calib_config, field)
    param = calib_config.(field);

    if ~all(alg.is_pos(param))
        field_class_error(field, param, 'positive array');
    end
end

function calib_config = validate_logical(calib_config, field)
    param = calib_config.(field);

    try
        calib_config.(field) = logical(param);
    catch
        field_class_error(field, param, 'logical array');
    end
end

function calib_config = validate_nan(calib_config, field)
    param = calib_config.(field);

    if ~all(isnan(param))
        field_class_error(field, param, 'nan array');
    end
end

function calib_config = validate_int_scalar(calib_config, field)
    calib_config = validate_scalar(calib_config, field);
    calib_config = validate_int(calib_config, field);
end

function calib_config = validate_logical_scalar(calib_config, field)
    calib_config = validate_scalar(calib_config, field);
    calib_config = validate_logical(calib_config, field);
end

function calib_config = validate_nan_scalar(calib_config, field)
    calib_config = validate_scalar(calib_config, field);
    calib_config = validate_nan(calib_config, field);
end

function calib_config = validate_pos_scalar(calib_config, field)
    calib_config = validate_scalar(calib_config, field);
    calib_config = validate_pos(calib_config, field);
end

function calib_config = validate_pos_int(calib_config, field)
    calib_config = validate_pos(calib_config, field);
    calib_config = validate_int(calib_config, field);
end

function calib_config = validate_pos_scalar_int(calib_config, field)
    calib_config = validate_scalar(calib_config, field);
    calib_config = validate_pos(calib_config, field);
    calib_config = validate_int(calib_config, field);
end

function calib_config = validate_pos_scalar_or_nan(calib_config, field)
    try
        calib_config = validate_pos_scalar(calib_config, field);
    catch
        calib_config = validate_nan_scalar(calib_config, field);
    end
end

function calib_config = validate_pos_scalar_int_or_nan(calib_config, field)
    try
        calib_config = validate_pos_scalar_int(calib_config, field);
    catch
        calib_config = validate_nan_scalar(calib_config, field);
    end
end

function calib_config = validate_logical_scalar_or_nan(calib_config, field)
    try
        calib_config = validate_logical_scalar(calib_config, field);
    catch
        calib_config = validate_nan_scalar(calib_config, field);
    end
end

function calib_config = validate_file(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if exist(param, 'file') ~= 2
        field_class_error(field, param, 'existing file');
    end
end

function calib_config = validate_interp(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if ~any(strcmp(param, {'linear', 'cubic', 'spline'}))
        field_class_error(field, param, 'interpolation type');
    end
end

function calib_config = validate_A_parameterization(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if ~any(strcmp(param, {'single_focal'}))
        field_class_error(field, param, 'A parameterization type');
    end
end

function calib_config = validate_R_parameterization(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    if ~any(strcmp(param, {'euler'}))
        field_class_error(field, param, 'R parameterization type');
    end
end

function calib_config = validate_sym_p_p2p_p_d(calib_config, field)
    param = calib_config.(field);

    % field needs to be a string
    calib_config = validate_string(calib_config, field);

    % If it starts with "distortion.", then its assumed to be a function in
    % +distortion, so just load it. Otherwise, its assumed to be a
    % "symbolic function string", so convert it.
    if startsWith(param, 'distortion.')
        [~, sym_distortion] = evalc(param);

        % Validate that this is indeed a symbolic function
        if ~isa(sym_distortion, 'symfun')
            field_class_error(field, param, 'symbolic function');
        end
    else
        sym_distortion = util.str2sym(param);
    end

    % Validate distortion function
    class.distortion.base.validate_sym_p_p2p_p_d(sym_distortion);

    % Assign value
    calib_config.(field) = sym_distortion;
end

function field_class_error(field, param, class_param)
    error(['Field: "' field '" has value: "' ...
           util.object_string(param) ...
           '" which is not a valid ' class_param '.']);
end
