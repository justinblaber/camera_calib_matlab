function calib = load_calib(file_path)
    % Loads a calibration from file path
    %
    % Inputs:
    %   file_path - string; path to load calibration
    %
    % Outputs:
    %   calib - struct; calibration

    % Check to make sure calibration file exists
    if exist(file_path, 'file') ~= 2
        error(['Calibration file: ' file_path ' does not exist.']);
    end

    % Read data
    data = util.read_data(file_path);

    % Parse calibration
    i = 1;
    calib_cams = struct('intrin', {}, 'extrin', {}, 'R_1', {}, 't_1', {});
    while isfield(data, ['A_cam' num2str(i)])
        % Parse single cameras
        [calib_cams(i), data] = util.parse_single_cam(data, ['_cam' num2str(i)]);

        % Increment
        i = i + 1;
    end

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
    calib.cam = calib_cams;
end
