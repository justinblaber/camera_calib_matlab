function cb_config = load_cb_config(cb_config_path)
    % Reads the calibration board config file given in cb_config_path. 
    % Outputs a struct containing calibration board info.
    % 
    % Inputs:
    %   cb_config_path - path to calibration board config file. Assumes 
    %       first line in config file is a header.
    %
    %   cb_config - struct containing the following fields:
    %       calibration - string; type of calibration
    %       four_point_height - scalar; height of the "four point" box
    %       four_point_width - scalar; width of the "four point" box
    %       num_rects_height - int; number of rectangles in the "height" 
    %           dimension
    %       num_rects_width - int; number of rectangles in the "width"
    %           dimension
    %       rect_height - scalar; height of an individual rectangle
    %       rect_width - scalar; width of an individual rectangle
    %       units - string; units in the calibration board

    % Check to make sure config file exists
    if exist(cb_config_path,'file') == 0
        error(['Config file: ' cb_config_path ' does not exist.']);
    end

    % Attempt to load config file
    try
        f = fopen(cb_config_path);
        C = textscan(f,'%s %s','delimiter',':','headerlines',1);
        fclose(f);
    catch e
        error(['Could not read config file; reason: ' getReport(e)]);
    end

    % Make sure, after text is delimited, that two columns of equal
    % length are loaded
    if ~isvector(C) || length(C) ~= 2 || ...
        ~isvector(C{1}) || ~isvector(C{2}) || ...
        length(C{1}) ~= length(C{2})
        disp_cb_config(cb_config_path);
        error(['config file does not contain a properly delimited ' ...
               'file. Please make sure each row has a single colon.'])
    end        

    % Store configuration
    cb_config = struct();
    for i = 1:size(C{1},1)
        cb_config.(C{1}{i}) = C{2}{i};
    end    
    
    % Validate fields exist:
    validate_field(cb_config,'calibration',cb_config_path);
    validate_field(cb_config,'four_point_height',cb_config_path);
    validate_field(cb_config,'four_point_width',cb_config_path);
    validate_field(cb_config,'num_rects_height',cb_config_path);
    validate_field(cb_config,'num_rects_width',cb_config_path);
    validate_field(cb_config,'rect_height',cb_config_path);
    validate_field(cb_config,'rect_width',cb_config_path);
    validate_field(cb_config,'units',cb_config_path);
    
    % Do additional validations
    validate_calibration(cb_config,cb_config_path);
    cb_config.four_point_height = validate_pos_num(cb_config,'four_point_height',cb_config_path);
    cb_config.four_point_width = validate_pos_num(cb_config,'four_point_width',cb_config_path);
    cb_config.num_rects_height = validate_pos_odd_int(cb_config,'num_rects_height',cb_config_path);
    cb_config.num_rects_width = validate_pos_odd_int(cb_config,'num_rects_width',cb_config_path);
    cb_config.rect_height = validate_pos_num(cb_config,'rect_height',cb_config_path);
    cb_config.rect_width = validate_pos_num(cb_config,'rect_width',cb_config_path);    
end

function disp_cb_config(cb_config_path)
    disp('Calibration board config file:');
    type(cb_config_path); % displays config file
end

function validate_field(cb_config,field,cb_config_path)
    if ~isfield(cb_config,field)
        disp_cb_config(cb_config_path);
        error(['Calibration field: ' field ' is missing from config file.']);
    end
end

function validate_calibration(cb_config,cb_config_path)
    switch cb_config.calibration
        case 'four_point_auto'
            disp('Using four_point_auto calibration.');
        case 'four_point_manual'
            disp('Using four_point_manual calibration.');
        otherwise
            disp_cb_config(cb_config_path);
            error(['Calibration type: ' cb_config.calibration ' is not supported.']);
    end
    disp('-----------------------------------------------');
end

function num = validate_pos_num(cb_config,field,cb_config_path)
    num = str2num(cb_config.(field)); %#ok<ST2NM>
    if isempty(num) || ~isscalar(num) || ~isreal(num) || ~isfinite(num) || num <= 0     
        disp_cb_config(cb_config_path);
        error(['Field: ' field ' has value: ' cb_config.(field) ' which is not a positive number.']);
    end
end

function num = validate_pos_odd_int(cb_config,field,cb_config_path)
    num = str2num(cb_config.(field)); %#ok<ST2NM>
    if isempty(num) || ~isscalar(num) || ~isreal(num) || ~isfinite(num) || num <= 0  || round(num) ~= num || mod(num,2) == 0
        disp_cb_config(cb_config_path);
        error(['Field: ' field ' has value: ' cb_config.(field) ' which is not a positive odd integer.']);
    end
end