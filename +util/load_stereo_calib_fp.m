function [calib, calib_config] = load_stereo_calib_fp(file_path)
    % Reads a four point stereo calibration from file path
    %
    % Inputs:
    %   file_path - string; path to calibration
    %
    % Outputs:
    %   calib - struct;
    %       .L - struct; calibration for left camera
    %       .R - struct; calibration for right camera
    %       .R_s - array; 3x3 rotation matrix describing rotation from
    %           left to right camera
    %       .t_s - array; 3x1 translation vector describing translation
    %           from left to right camera
    %   calib_config - struct; struct returned by util.read_calib_config()

    % Check to make sure data file exists
    if exist(file_path, 'file') ~= 2
        error(['Calibration file: ' file_path ' does not exist.']);
    end

    % Read data
    data = util.read_data(file_path);

    % Parse out the left calibration
    [calib.L, data] = util.parse_single_calib_fp(data, '_L');

    % Parse out the right calibration
    [calib.R, data] = util.parse_single_calib_fp(data, '_R');
        
    % Parse R_s and t_s
    [calib.R_s, data] = util.read_and_remove(data, 'R_s');
    [calib.t_s, data] = util.read_and_remove(data, 't_s');

    % Parse out calib config
    [calib_config, data] = util.parse_calib_config(data);
    
    % Display error if there are still fields existing in data
    fields_data = fields(data);
    if numel(fields_data) ~= 0
        error(['When reading calibration, the following unknown ' ...
               'fields were found: "' strjoin(fields_data, ', ') '".']);
    end
end
