function cb_config = load_cb_config(cb_config_path)
    % Reads the calibration board config file given in cb_config_path. 
    % Outputs a struct containing calibration board info.
    % 
    % Inputs:
    %   cb_config_path - path to calibration board config file. Assumes 
    %       first line in config file is a header (for yaml).
    %
    % Outputs:
    %   cb_config - struct containing the following fields:
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
    %       homography_it_cutoff - int; number of iterations performed for 
    %           nonlinear homography refinement
    %       homography_norm_cutoff = scalar; cutoff for norm of difference
    %           of parameter vector for nonlinear homography refinement
    %
    %       refine_it_cutoff - int; number of iterations performed for 
    %           refinement of corner point
    %       refine_norm_cutoff = scalar; cutoff for the difference in
    %           corner point position for corner point refinement
    %       refine_default_window_factor - scalar; default relative size of
    %           corner refinement window compared to size of the 
    %           calibration board squares.
    %       refine_window_min_size - scalar; minimum length of refinement 
    %           window. This will recompute the window_factor to meet the 
    %           minimum specified length.

    % Check to make sure config file exists
    if exist(cb_config_path,'file') == 0
        error(['Config file: ' cb_config_path ' does not exist.']);
    end
    
    % Display contents of config file    
    disp('Calibration board config file:');
    type(cb_config_path); % displays config file
    disp(' ');
    disp('--------------------------------------------');
    
    % Attempt to load config file
    try
        f = fopen(cb_config_path);
        C = textscan(f,'%s %s','delimiter','=','headerlines',1);
        fclose(f);
    catch e
        error(['Could not read config file; reason: ' getReport(e)]);
    end
       
    % Make sure, after text is delimited, that two columns of equal
    % length are loaded
    if ~isvector(C) || length(C) ~= 2 || ...
        ~isvector(C{1}) || ~isvector(C{2}) || ...
        length(C{1}) ~= length(C{2})
        error(['Config file does not contain a properly delimited ' ...
               'file. Please make sure each row has a single equal sign.'])
    end      
    
    % Remove comments - make sure '%%'' is the first substring found
    comment_idx = cellfun(@(x)ismember(1,x),strfind(C{1},'%%'));
    C{1}(comment_idx) = [];
    C{2}(comment_idx) = [];    
    
    % Store configuration
    cb_config = struct();
    for i = 1:size(C{1},1)
        cb_config.(strtrim(C{1}{i})) = strtrim(C{2}{i});
    end    
    
    % Perform validations on input fields    
    % Required fields
    field_info        = struct('field','num_squares_height'          ,'required',true ,'default',''    ,'validation_fun',@validate_pos_odd_int);
    field_info(end+1) = struct('field','num_squares_width'           ,'required',true ,'default',''    ,'validation_fun',@validate_pos_odd_int);
    field_info(end+1) = struct('field','square_size'                 ,'required',true ,'default',''    ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','units'                       ,'required',true ,'default',''    ,'validation_fun',[]);
    field_info(end+1) = struct('field','calibration'                 ,'required',true ,'default',''    ,'validation_fun',@validate_calibration);
    field_info(end+1) = struct('field','four_point_height'           ,'required',true ,'default',''    ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','four_point_width'            ,'required',true ,'default',''    ,'validation_fun',@validate_pos_num);
    % Optional fields
    field_info(end+1) = struct('field','homography_it_cutoff'        ,'required',false,'default','5'   ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','homography_norm_cutoff'      ,'required',false,'default','1e-5','validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_it_cutoff'            ,'required',false,'default','10'  ,'validation_fun',@validate_pos_int);
    field_info(end+1) = struct('field','refine_norm_cutoff'          ,'required',false,'default','0.05','validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_default_window_factor','required',false,'default','2/3' ,'validation_fun',@validate_pos_num);
    field_info(end+1) = struct('field','refine_window_min_size'      ,'required',false,'default','10'  ,'validation_fun',@validate_pos_num);
    
    % Check to see if any unrecognized fields exist
    cb_config_fields = fields(cb_config);
    for i = 1:length(cb_config_fields)
        if ~any(strcmp(cb_config_fields{i},{field_info.field}))
            error(['Unrecognized field: "' cb_config_fields{i} '" ' ...
                   'in calibration board config file.']);
        end        
    end
    
    % Validate all inputs
    for i = 1:length(field_info)
        % Check for required fields; for optional fields that dont exist, 
        % set the default value
        if ~any(strcmp(field_info(i).field,cb_config_fields))            
            if field_info(i).required
                error(['Required field: "' field_info(i).field '" was ' ...
                       'not found in calibration board config file.']);
            else
                % This is an optional field; set the default value since it
                % was not set
                cb_config.(field_info(i).field) = field_info(i).default;
            end            
        end
        
        % Validate field if a validation function was set
        if ~isempty(field_info(i).validation_fun)
            cb_config = field_info(i).validation_fun(cb_config,field_info(i).field);
        end
    end
end

% Make all validate_* functions take cb_config and the field, and return
% the cb_config. This makes things easier.

function cb_config = validate_calibration(cb_config,field)
    switch cb_config.(field)
        case 'four_point_auto'
        case 'four_point_manual'
        otherwise
            error(['Calibration type: "' cb_config.calibration '" is not supported.']);
    end
end

function cb_config = validate_pos_num(cb_config,field)
    num = str2num(cb_config.(field)); %#ok<ST2NM>
    if ~util.is_pos(num) 
        error(['Field: ' field ' has value: "' cb_config.(field) '" which is not a positive number.']);
    end
    cb_config.(field) = num;
end

function cb_config = validate_pos_int(cb_config,field)
    num = str2num(cb_config.(field)); %#ok<ST2NM>
    if ~util.is_pos(num) || ~util.is_int(num)
        error(['Field: ' field ' has value: "' cb_config.(field) '" which is not a positive integer.']);
    end
    cb_config.(field) = num;
end

function cb_config = validate_pos_odd_int(cb_config,field)
    num = str2num(cb_config.(field)); %#ok<ST2NM>
    if ~util.is_pos(num) || ~util.is_int(num) || util.is_even(num)
        error(['Field: ' field ' has value: "' cb_config.(field) '" which is not a positive odd integer.']);
    end
    cb_config.(field) = num;
end