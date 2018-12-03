function [calib_config, data] = parse_calib_config(data)
    
    % Set field info -----------------------------------------------------%
    
    % Calibration board info
    field_info        = struct('field','target_type'                                ,'required',true ,'default',''                             ,'validation_fun',@validate_target_type);
    field_info(end+1) = struct('field','num_targets_height'                         ,'required',true ,'default',[]                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','num_targets_width'                          ,'required',true ,'default',[]                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','target_spacing'                             ,'required',true ,'default',[]                             ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','units'                                      ,'required',true ,'default',''                             ,'validation_fun',@validate_string);
    field_info(end+1) = struct('field','height_fp'                                  ,'required',true ,'default',[]                             ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','width_fp'                                   ,'required',true ,'default',[]                             ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','target_mat'                                 ,'required',false,'default',[]                             ,'validation_fun',@validate_target_mat);
    
    % Verbosity
    field_info(end+1) = struct('field','verbosity'                                  ,'required',false,'default',3                              ,'validation_fun',@validate_int);
    
    % Homography computation
    field_info(end+1) = struct('field','homography_p2p_it_cutoff'                   ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','homography_p2p_norm_cutoff'                 ,'required',false,'default',1e-6                           ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','homography_c2e_it_cutoff'                   ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','homography_c2e_norm_cutoff'                 ,'required',false,'default',1e-6                           ,'validation_fun',@validate_pos_num);
    
    % Checker refinement
    field_info(end+1) = struct('field','refine_checker_min_hw'                      ,'required',false,'default',4                              ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_checker_max_hw'                      ,'required',false,'default',15                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_checker_opencv_it_cutoff'            ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_checker_opencv_norm_cutoff'          ,'required',false,'default',0.001                          ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_checker_edges_it_cutoff'             ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_checker_edges_norm_cutoff'           ,'required',false,'default',0.001                          ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_checker_edges_h2_init'               ,'required',false,'default',0.75                           ,'validation_fun',@validate_pos_num);
    
    % Ellipse refinement
    field_info(end+1) = struct('field','circle_radius'                              ,'required',false,'default',nan                            ,'validation_fun',@validate_circle_radius);
    field_info(end+1) = struct('field','refine_ellipse_edges_it_cutoff'             ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_ellipse_edges_norm_cutoff'           ,'required',false,'default',0.001                          ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','refine_ellipse_edges_h2_init'               ,'required',false,'default',0.75                           ,'validation_fun',@validate_pos_num);    
    
    % p_p_d2p_p
    field_info(end+1) = struct('field','p_p_d2p_p_it_cutoff'                        ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','p_p_d2p_p_norm_cutoff'                      ,'required',false,'default',1e-6                           ,'validation_fun',@validate_pos_num);
    
    % Undistort array
    field_info(end+1) = struct('field','undistort_array_interp'                     ,'required',false,'default','spline'                       ,'validation_fun',@validate_interp);
	
    % Distort array
    field_info(end+1) = struct('field','distort_array_interp'                       ,'required',false,'default','spline'                       ,'validation_fun',@validate_interp);
    
    % Distortion refinement
    field_info(end+1) = struct('field','distortion_refinement_it_cutoff'            ,'required',false,'default',5                              ,'validation_fun',@validate_pos_int);
    
    % Covariance optimization
    field_info(end+1) = struct('field','apply_covariance_optimization'              ,'required',false,'default',true                           ,'validation_fun',@validate_logical);
    
    % sym_p_p2p_p_d
    field_info(end+1) = struct('field','sym_p_p2p_p_d'                              ,'required',false,'default','distortion.wang08'            ,'validation_fun',@validate_distortion);
    
    % Single calibration    
    field_info(end+1) = struct('field','refine_single_params_it_cutoff'             ,'required',false,'default',200                            ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_single_params_norm_cutoff'           ,'required',false,'default',1e-6                           ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','refine_single_params_lambda_init'           ,'required',false,'default',0.01                           ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','refine_single_params_lambda_factor'         ,'required',false,'default',10                             ,'validation_fun',@validate_pos_num);    
    
    % Stereo calibration  
    field_info(end+1) = struct('field','refine_stereo_params_it_cutoff'             ,'required',false,'default',200                            ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_stereo_params_norm_cutoff'           ,'required',false,'default',1e-6                           ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','refine_stereo_params_lambda_init'           ,'required',false,'default',0.01                           ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','refine_stereo_params_lambda_factor'         ,'required',false,'default',10                             ,'validation_fun',@validate_pos_num);    
    
    % Blob detection
    field_info(end+1) = struct('field','blob_detect_r_range1'                       ,'required',false,'default',1                              ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_r_range2'                       ,'required',false,'default',15                             ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_step'                           ,'required',false,'default',0.5                            ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_num_cutoff'                     ,'required',false,'default',1000                           ,'validation_fun',@validate_pos_int);    
    field_info(end+1) = struct('field','blob_detect_LoG_cutoff'                     ,'required',false,'default',0.1                            ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_LoG_interp'                     ,'required',false,'default','cubic'                        ,'validation_fun',@validate_interp);    
    field_info(end+1) = struct('field','blob_detect_eccentricity_cutoff'            ,'required',false,'default',5                              ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_lambda'                         ,'required',false,'default',1e-2                           ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_maxima_it_cutoff'               ,'required',false,'default',10                             ,'validation_fun',@validate_pos_int);    
    field_info(end+1) = struct('field','blob_detect_maxima_norm_cutoff'             ,'required',false,'default',1e-6                           ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_centroid_it_cutoff'             ,'required',false,'default',10                             ,'validation_fun',@validate_pos_int);    
    field_info(end+1) = struct('field','blob_detect_centroid_norm_cutoff'           ,'required',false,'default',0.1                            ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_d_cluster'                      ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_r1_cluster'                     ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num);    
    field_info(end+1) = struct('field','blob_detect_r2_cluster'                     ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num);    
        
    % Four point detection    
    field_info(end+1) = struct('field','ellipse_detect_num_samples_theta'           ,'required',false,'default',100                            ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','ellipse_detect_interp'                      ,'required',false,'default','cubic'                        ,'validation_fun',@validate_interp); 
    field_info(end+1) = struct('field','ellipse_detect_sf_cost'                     ,'required',false,'default',2                              ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','ellipse_detect_it_cutoff'                   ,'required',false,'default',100                            ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','ellipse_detect_norm_cutoff'                 ,'required',false,'default',1e-3                           ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_lambda_init'                 ,'required',false,'default',1                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_lambda_factor'               ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_d_cluster'                   ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_r1_cluster'                  ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_r2_cluster'                  ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num);  
    field_info(end+1) = struct('field','four_points_detect_marker_templates_path'   ,'required',false,'default','+markers/marker_templates.txt','validation_fun',@validate_file);   
    field_info(end+1) = struct('field','four_points_detect_marker_config_path'      ,'required',false,'default','+markers/marker.conf'         ,'validation_fun',@validate_file);   
    field_info(end+1) = struct('field','four_points_detect_num_cutoff'              ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','four_points_detect_mse_cutoff'              ,'required',false,'default',0.2                            ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','four_points_detect_padding_radial'          ,'required',false,'default',5                              ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','four_points_detect_array_min_size'          ,'required',false,'default',400                            ,'validation_fun',@validate_int); 
    
    % Plotting info
    field_info(end+1) = struct('field','camera_size'                                ,'required',false,'default',0                              ,'validation_fun',@validate_num);
       
    % Create calib_config ------------------------------------------------%
    
    % Validate all inputs
    data_fields = fields(data);
    for i = 1:numel(field_info)        
        % Check if field exists in data
        if any(strcmp(field_info(i).field, data_fields))
            % Field exists; get the value and remove it
            [param, data] = util.read_and_remove(data, ...
                                                 field_info(i).field);
            
            % Set value
            calib_config.(field_info(i).field) = param;
        else
            % Field doesn't exist; check if its required or not
            if field_info(i).required
                error(['Required field: "' field_info(i).field '" was ' ...
                       'not found.']);
            else
                % This is an optional field; set the default value since it
                % was not set
                calib_config.(field_info(i).field) = field_info(i).default;
            end            
        end
        
        % Validate field if a validation function was set
        if ~isempty(field_info(i).validation_fun)
            calib_config = field_info(i).validation_fun(calib_config, ...
                                                        field_info(i).field);
        end
    end  
end

% Make all validate_* functions take calib_config and the field, and return
% the calib_config. This makes things easier.

function calib_config = validate_target_type(calib_config,field)
    if ~ischar(calib_config.(field))
        field_struct_class_error(field, calib_config, 'string');
    end
    
    if ~any(strcmp(calib_config.(field),{'checker','circle'}))
        field_struct_class_error(field, calib_config, 'calibration target');
    end
end

function calib_config = validate_int(calib_config,field)
    if ~util.is_int(calib_config.(field))
        field_struct_class_error(field, calib_config, 'integer');
    end
end

function calib_config = validate_num(calib_config,field)
    if ~util.is_num(calib_config.(field)) 
        field_struct_class_error(field, calib_config, 'number');
    end
end

function calib_config = validate_pos_int(calib_config,field)
    if ~util.is_pos(calib_config.(field)) || ~util.is_int(calib_config.(field))
        field_struct_class_error(field, calib_config, 'positive integer');
    end
end

function calib_config = validate_pos_num(calib_config,field)
    if ~util.is_pos(calib_config.(field)) 
        field_struct_class_error(field, calib_config, 'positive number');
    end
end

function calib_config = validate_string(calib_config,field)
    if ~ischar(calib_config.(field)) 
        field_struct_class_error(field, calib_config, 'string');
    end
end

function calib_config = validate_logical(calib_config,field)
    % Try to convert to boolean
    try 
        calib_config.(field) = logical(calib_config.(field));
    catch
        field_struct_class_error(field, calib_config, 'boolean');
    end

    % Make sure output is a scalar
    if ~isscalar(calib_config.(field))
        field_struct_class_error(field, calib_config, 'boolean scalar');
    end
end

function calib_config = validate_interp(calib_config,field)
    if ~ischar(calib_config.(field)) 
        field_struct_class_error(field, calib_config, 'string');
    end
    
    if ~any(strcmp(calib_config.(field),{'linear','cubic','spline'}))
        field_struct_class_error(field, calib_config, 'interpolation type');
    end
end

function calib_config = validate_target_mat(calib_config,field)
    % If not set, initialize to all true
    if isempty(calib_config.(field))
        calib_config.(field) = true(calib_config.num_targets_height, ...
                                    calib_config.num_targets_width);
    end
        
    % Try to convert to boolean
    try 
        calib_config.(field) = logical(calib_config.(field));
    catch
        field_struct_class_error(field, calib_config, 'boolean');
    end
        
    % Validate size of target_mat
    if size(calib_config.(field),1) ~= calib_config.num_targets_height || ...
       size(calib_config.(field),2) ~= calib_config.num_targets_width
        field_struct_class_error(field, calib_config, 'size');
    end
end

function calib_config = validate_circle_radius(calib_config,field)
    % Check if target type is a circle
    if strcmp(calib_config.target_type,'circle')
        % Make sure circle_radius is set to a positive number
        if isnan(calib_config.(field))
            field_struct_class_error(field, calib_config, 'value; it must be set.');
        elseif ~util.is_pos(calib_config.(field))
            field_struct_class_error(field, calib_config, 'positive number');
        end
    end
end

function calib_config = validate_distortion(calib_config,field)
    % Make sure field is a string starting with "distortion."
    if ~ischar(calib_config.(field)) || ~startsWith(calib_config.(field),'distortion.')
        field_struct_class_error(field, calib_config, 'value; it must start with "distortion."');
    end
    
    % This will convert string to symbolic function
    sym_distortion = eval(calib_config.(field));
    
    % Validate that this is indeed a symbolic function
    if ~isa(sym_distortion,'symfun')
        field_struct_class_error(field, calib_config, 'symbolic function');
    end
    
    % Validate that this is a valid distortion function
    args = arrayfun(@char,argnames(sym_distortion),'UniformOutput',false);
    if ~all(strcmp(args(1:5),{'x_p','y_p','a','x_o','y_o'}))
        field_struct_class_error(field, calib_config, 'distortion function; it must have that has arguments which start with (x_p,y_p,a,x_o,y_o)');
    end
end

function calib_config = validate_file(calib_config,field)
    if ~ischar(calib_config.(field)) || exist(calib_config.(field),'file') ~= 2
        field_struct_class_error(field, calib_config, 'existing file.');
    end
end

function field_struct_class_error(field, data, class_param)
    error(['Field: "' field '" has a value: "' ...
           util.object_string(data.(field)) ...
           '" which is not a valid ' class_param '.']);
end