function calib = load_stereo_calib(file_path)
    % Loads a four point stereo calibration from file path
    %
    % Inputs:
    %   file_path - string; path to calibration
    %
    % Outputs:
    %   calib - struct;
    %       .config - struct; calibration config
    %       .L - struct; calibration for left camera
    %       .R - struct; calibration for right camera
    %       .R_s - array; 3x3 rotation matrix describing rotation from
    %           left to right camera
    %       .t_s - array; 3x1 translation vector describing translation
    %           from left to right camera

    % Check to make sure data file exists
    if exist(file_path, 'file') ~= 2
        error(['Calibration file: ' file_path ' does not exist.']);
    end

    % Read data
    data = util.read_data(file_path);

    % Parse out the left calibration
    [calib_L, data] = util.parse_single_calib(data, '_L');

    % Parse out the right calibration
    [calib_R, data] = util.parse_single_calib(data, '_R');

    % Parse R_s and t_s
    [R_s, data] = util.read_and_remove(data, 'R_s');
    [t_s, data] = util.read_and_remove(data, 't_s');

    % Parse out calib config
    [calib_config, data] = util.parse_calib_config(data);

    % Display error if there are still fields existing in data
    fields_data = fields(data);
    if numel(fields_data) ~= 0
        error(['When reading calibration, the following unknown ' ...
               'fields were found: "' strjoin(fields_data, ', ') '".']);
    end

    % Package outputs
    calib.config = calib_config;
    calib.L = calib_L;
    calib.R = calib_R;
    calib.R_s = R_s;
    calib.t_s = t_s;
end
