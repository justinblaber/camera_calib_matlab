function calib = read_single_calib_fp(file_path)
                
    % TODO: add checks to validate file
    
    % Check to make sure data file exists
    if exist(file_path,'file') == 0
        error(['Calibration file: ' file_path ' does not exist.']);
    end
    
    % Read data
    data = util.read_data(file_path);
    
    % Parse out single four point calibration
    [calib, data] = util.parse_single_calib_fp(data);
        
    % Parse out calib config
    [calib.config, data] = util.parse_calib_config(data);
        
    % Display error if there are still fields existing in data
    fields_data = fields(data);
    if numel(fields_data) ~= 0
        error(['When reading calibration, the following unknown ' ...
               'fields were found: "' strjoin(fields_data,', ') '".']);
    end
end