function calib_config = read_calib_config(calib_config_path)
    % Reads the calibration config file given in calib_config_path and 
    % outputs a struct containing calibration related info.
    % 
    % Inputs:
    %   calib_config_path - string; path to calibration config file.
    %
    % Outputs:
    %   calib_config - struct;

    % Check to make sure config file exists
    if exist(calib_config_path,'file') == 0
        error(['Config file: "' calib_config_path '" does not exist.']);
    end

    % Load config file
    calib_config = util.read_data(calib_config_path);
    
    % Perform validations on input fields --------------------------------%
    
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
    field_info(end+1) = struct('field','undistort_array_interp'                     ,'required',false,'default','spline'                       ,'validation_fun',@validate_string);
	
    % Distort array
    field_info(end+1) = struct('field','distort_array_interp'                       ,'required',false,'default','spline'                       ,'validation_fun',@validate_string);
    
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
    field_info(end+1) = struct('field','blob_detect_LoG_interp'                     ,'required',false,'default','cubic'                        ,'validation_fun',@validate_string);    
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
    field_info(end+1) = struct('field','ellipse_detect_interp'                      ,'required',false,'default','cubic'                        ,'validation_fun',@validate_string); 
    field_info(end+1) = struct('field','ellipse_detect_sf_cost'                     ,'required',false,'default',2                              ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','ellipse_detect_it_cutoff'                   ,'required',false,'default',100                            ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','ellipse_detect_norm_cutoff'                 ,'required',false,'default',1e-3                           ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_lambda_init'                 ,'required',false,'default',1                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_lambda_factor'               ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_d_cluster'                   ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_r1_cluster'                  ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','ellipse_detect_r2_cluster'                  ,'required',false,'default',2                              ,'validation_fun',@validate_pos_num);  
    field_info(end+1) = struct('field','four_points_detect_marker_templates_path'   ,'required',false,'default','+markers/marker_templates.txt','validation_fun',@validate_path);   
    field_info(end+1) = struct('field','four_points_detect_marker_config_path'      ,'required',false,'default','+markers/marker.conf'         ,'validation_fun',@validate_path);   
    field_info(end+1) = struct('field','four_points_detect_num_cutoff'              ,'required',false,'default',20                             ,'validation_fun',@validate_pos_int); 
    field_info(end+1) = struct('field','four_points_detect_mse_cutoff'              ,'required',false,'default',0.2                            ,'validation_fun',@validate_pos_num); 
    field_info(end+1) = struct('field','four_points_detect_padding_radial'          ,'required',false,'default',5                              ,'validation_fun',@validate_pos_int); 
    
    % Plotting info
    field_info(end+1) = struct('field','camera_size'                                ,'required',false,'default',0                              ,'validation_fun',@validate_num);
        
    % Check to see if any unrecognized fields exist
    calib_config_fields = fields(calib_config);
    for i = 1:numel(calib_config_fields)
        if ~any(strcmp(calib_config_fields{i},{field_info.field}))
            error(['Unrecognized field: "' calib_config_fields{i} '" ' ...
                   'in calibration config file.']);
        end        
    end
    
    % Validate all inputs
    for i = 1:numel(field_info)
        % For optional fields that dont exist, set the default value
        if ~any(strcmp(field_info(i).field,calib_config_fields))            
            if field_info(i).required
                error(['Required field: "' field_info(i).field '" was ' ...
                       'not found in calibration config file.']);
            else
                % This is an optional field; set the default value since it
                % was not set
                calib_config.(field_info(i).field) = field_info(i).default;
            end            
        end
        
        % Validate field if a validation function was set
        if ~isempty(field_info(i).validation_fun)
            calib_config = field_info(i).validation_fun(calib_config,field_info(i).field);
        end
    end   
    
    % Display contents of config file
    util.verbose_disp('------------',1,calib_config);
    util.verbose_disp('Calibration config file:',1,calib_config);
    util.verbose_disp(' ',1,calib_config);
    util.verbose_disp(fileread(calib_config_path),1,calib_config);
    util.verbose_disp(' ',1,calib_config);
end

% Make all validate_* functions take calib_config and the field, and return
% the calib_config. This makes things easier.

function calib_config = validate_target_type(calib_config,field)
    switch calib_config.(field)
        case 'checker'
        case 'circle'
        otherwise
            error(['Calibration target: "' calib_config.(field) '" is not supported.']);
    end
end

function calib_config = validate_int(calib_config,field)
    if ~util.is_int(calib_config.(field))
        error(['Field: "' field '" has a value which is not an integer.']);
    end
end

function calib_config = validate_num(calib_config,field)
    if ~util.is_num(calib_config.(field)) 
        error(['Field: "' field '" has a value which is not a number.']);
    end
end

function calib_config = validate_pos_int(calib_config,field)
    if ~util.is_pos(calib_config.(field)) || ~util.is_int(calib_config.(field))
        error(['Field: "' field '" has a value which is not a positive integer.']);
    end
end

function calib_config = validate_pos_num(calib_config,field)
    if ~util.is_pos(calib_config.(field)) 
        error(['Field: "' field '" has a value which is not a positive number.']);
    end
end

function calib_config = validate_string(calib_config,field)
    if ~ischar(calib_config.(field)) 
        error(['Field: "' field '" has a value which is not a string.']);
    end
end

function calib_config = validate_logical(calib_config,field)
    if ~islogical(calib_config.(field)) 
        error(['Field: "' field '" has a value which is not a logical.']);
    end
end

function calib_config = validate_target_mat(calib_config,field)    
    if isempty(calib_config.(field))
        % If not set, initialize to all true
        calib_config.(field) = true(calib_config.num_targets_height, ...
                                    calib_config.num_targets_width);
    end
    
    % Validate size of target_mat
    if size(calib_config.(field),1) ~= calib_config.num_targets_height || ...
       size(calib_config.(field),2) ~= calib_config.num_targets_width
        error(['Field: "' field '" has an invalid size of: ' num2str(size(calib_config.(field)))]);
    end        
    
    % Ensure target_mat is boolean
    calib_config.(field) = logical(calib_config.(field));
end

function calib_config = validate_circle_radius(calib_config,field)
    if strcmp(calib_config.target_type,'circle') && isnan(calib_config.(field))
        error(['Field: "' field '" must be set when using circle targets.']);
    elseif strcmp(calib_config.target_type,'circle') && calib_config.(field) <= 0
        error(['Field: "' field '" must be a positive value.']);
    end
end

function calib_config = validate_distortion(calib_config,field)
    if ~startsWith(calib_config.(field),'distortion.')
        error(['Field: "' field '" must start with ''distortion.''.']);
    end

    calib_config.(field) = eval(calib_config.(field));
end

function calib_config = validate_path(calib_config,field)
    if isempty(calib_config.(field)) || exist(calib_config.(field),'file') ~= 2
        error(['Field: "' field '" has a value which is not an existing file.']);
    end
end