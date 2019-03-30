function calib = load_calib(file_path)

    % Check to make sure data file exists
    if exist(file_path, 'file') ~= 2
        error(['Calibration file: ' file_path ' does not exist.']);
    end

    % Read data
    data = util.read_data(file_path);

    % Parse calibration
    i = 1;
    while isfield(data, ['A_cam' num2str(i)])
        % Parse single calibration
        [calib_cams(i), data] = util.parse_single_calib(data, ['_cam' num2str(i)]); %#ok<AGROW>

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
    for i = 1:numel(calib_cams)
        calib.cam(i) = calib_cams(i);
    end
end
