function calib_config = read_calib_config(calib_config_path)
    % Reads the calibration config file given in calib_config_path. 
    % Outputs a struct containing calibration related info.
    % 
    % Inputs:
    %   calib_config_path - path to calibration config file.
    %
    % Outputs:
    %   calib_config - struct containing the following fields:
    %
    %       Calibration board info:    
    %
    %       num_squares_height - int; number of squares in the "height" 
    %           dimension
    %       num_squares_width - int; number of squares in the "width"
    %           dimension
    %       square_size - scalar; length of the side of the square
    %       units - string; units of square size
    %
    %       calibration - string; type of calibration
    %       four_point_height - scalar; height of the "four point" box
    %       four_point_width - scalar; width of the "four point" box
    %
    %       Algorithmic info:
    %
    %       verbose - int; level of verbosity in output.
    %
    %       homography_it_cutoff - int; number of iterations performed for 
    %           nonlinear homography refinement
    %       homography_norm_cutoff - scalar; cutoff for norm of difference
    %           of parameter vector for nonlinear homography refinement
    %
    %       refine_corner_it_cutoff - int; number of iterations performed 
    %           for refinement of corner point
    %       refine_corner_norm_cutoff - scalar; cutoff for the difference 
    %           in corner point position for corner point refinement
    %       refine_corner_default_window_factor - scalar; default relative
    %           size of corner refinement window compared to size of the 
    %           calibration board squares.
    %       refine_corner_window_min_size - scalar; minimum length of 
    %           refinement window. This will recompute the window_factor to
    %           meet the minimum specified length.
    %
    %       refine_param_it_cutoff - int; number of iterations performed 
    %           for refinement of calibration parameters
    %       refine_param_norm_cutoff - scalar; cutoff for the difference in
    %           norm of calibration parameters
    %
    %       blob_detect_s - int; number of partitions of scale space
    %       blob_detect_num_octaves - int; number of octaves of scale space
    %       blob_detect_contrast_cutoff - scalar; cutoff for DoG contrast
    %       blob_detect_r_cutoff - scalar; cutoff for r parameter described
    %           in Lowe's SIFT paper
    %       blob_detect_second_moment_cutoff - scalar; cutoff for ratio of
    %           sqrt of max and min eigenvalues of second moment matrix
    %
    %       marker_config_path - string; path to marker configuration
    %       marker_templates_path - string; path to marker_templates
    %
    %       Plotting info:
    %
    %       camera_size - scalar; size of camera in specified units; used
    %           for plotting extrinsic parameters.

    % Check to make sure config file exists
    if exist(calib_config_path,'file') == 0
        error(['Config file: ' calib_config_path ' does not exist.']);
    end
    
    % Display contents of config file    
    disp('--------------------------------------------');
    disp('Calibration config file:');
    type(calib_config_path);
    disp(' ');
    
    % Load config file
    calib_config = util.read_data(calib_config_path);
    
    % Perform validations on input fields    
    % Calibration board info
    field_info        = struct('field','num_squares_height'                 ,'required',true ,'default',''  ,'validation_fun',@validate_pos_odd_int);
    field_info(end+1) = struct('field','num_squares_width'                  ,'required',true ,'default',''  ,'validation_fun',@validate_pos_odd_int);
    field_info(end+1) = struct('field','square_size'                        ,'required',true ,'default',''  ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','units'                              ,'required',true ,'default',''  ,'validation_fun',@validate_string);
    field_info(end+1) = struct('field','calibration'                        ,'required',true ,'default',''  ,'validation_fun',@validate_calibration);
    field_info(end+1) = struct('field','four_point_height'                  ,'required',true ,'default',''  ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','four_point_width'                   ,'required',true ,'default',''  ,'validation_fun',@validate_pos_num);
    % Algorithmic info
    field_info(end+1) = struct('field','verbose'                            ,'required',false,'default',1   ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','homography_it_cutoff'               ,'required',false,'default',10  ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','homography_norm_cutoff'             ,'required',false,'default',1e-6,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_corner_it_cutoff'            ,'required',false,'default',10  ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_corner_norm_cutoff'          ,'required',false,'default',0.05,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_corner_default_window_factor','required',false,'default',2/3 ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_corner_window_min_size'      ,'required',false,'default',10  ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_param_it_cutoff'             ,'required',false,'default',20  ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_param_norm_cutoff'           ,'required',false,'default',1e-6,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','blob_detect_s'                      ,'required',false,'default',2   ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','blob_detect_num_octaves'            ,'required',false,'default',4   ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','blob_detect_contrast_cutoff'        ,'required',false,'default',0.03,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','blob_detect_r_cutoff'               ,'required',false,'default',10  ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','blob_detect_second_moment_cutoff'   ,'required',false,'default',3   ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','marker_config_path'                 ,'required',false,'default',''  ,'validation_fun',@validate_file_path);
    field_info(end+1) = struct('field','marker_templates_path'              ,'required',false,'default',''  ,'validation_fun',@validate_file_path);
        
    % Plotting info
    field_info(end+1) = struct('field','camera_size'                        ,'required',false,'default',eps ,'validation_fun',@validate_pos_num);
    
    % Check to see if any unrecognized fields exist
    calib_config_fields = fields(calib_config);
    for i = 1:length(calib_config_fields)
        if ~any(strcmp(calib_config_fields{i},{field_info.field}))
            error(['Unrecognized field: "' calib_config_fields{i} '" ' ...
                   'in calibration config file.']);
        end        
    end
    
    % Validate all inputs
    for i = 1:length(field_info)
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
end

% Make all validate_* functions take calib_config and the field, and return
% the calib_config. This makes things easier.

function calib_config = validate_calibration(calib_config,field)
    try
        switch calib_config.(field)
            case 'four_point_auto'
            case 'four_point_manual'
            otherwise
                error('Calibration type is not supported.');
        end
    catch
        error('Calibration type is not supported.');
    end
end

function calib_config = validate_string(calib_config,field)
    if ~ischar(calib_config.(field)) 
        error(['Field: ' field ' has a value which is not a string.']);
    end
    calib_config.(field) = calib_config.(field);
end

function calib_config = validate_pos_num(calib_config,field)
    if ~util.is_pos(calib_config.(field)) 
        error(['Field: ' field ' has a value which is not a positive number.']);
    end
    calib_config.(field) = calib_config.(field);
end

function calib_config = validate_pos_int(calib_config,field)
    if ~util.is_pos(calib_config.(field)) || ~util.is_int(calib_config.(field))
        error(['Field: ' field ' has a value which is not a positive integer.']);
    end
    calib_config.(field) = calib_config.(field);
end

function calib_config = validate_pos_odd_int(calib_config,field)
    if ~util.is_pos(calib_config.(field)) || ~util.is_int(calib_config.(field)) || util.is_even(calib_config.(field))
        error(['Field: ' field ' has a value which is not a positive odd integer.']);
    end
    calib_config.(field) = calib_config.(field);
end

function calib_config = validate_file_path(calib_config,field)
    % This needs to either be empty, or if its not empty, the file needs to
    % exist
    if ~isempty(calib_config.(field)) && exist(calib_config.(field),'file') ~= 2
        error(['Field: ' field ' has a value which is not an existing file.']);
    end
    calib_config.(field) = calib_config.(field);
end
