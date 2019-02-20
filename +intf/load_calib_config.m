function calib_config = load_calib_config(calib_config_path)
    % Loads the calibration config file given in calib_config_path and
    % outputs a struct containing calibration related info.
    %
    % Inputs:
    %   calib_config_path - string; path to calibration config file.
    %
    % Outputs:
    %   calib_config - struct;

    % Check to make sure config file exists
    if exist(calib_config_path, 'file') ~= 2
        error(['Config file: "' calib_config_path '" does not exist.']);
    end

    % Load data
    data = util.read_data(calib_config_path);

    % Parse it
    [calib_config, data] = util.parse_calib_config(data);

    % Display error if there are still fields existing in data
    fields_data = fields(data);
    if numel(fields_data) ~= 0
        error(['When reading calibration config, the following unknown ' ...
               'fields were found: "' strjoin(fields_data, ', ') '".']);
    end

    % Display contents of config file if verbosity is set
    util.verbose_disp('------------', 1, calib_config);
    util.verbose_disp('Calibration config file:', 1, calib_config);
    util.verbose_disp(' ', 1, calib_config);
    util.verbose_disp(fileread(calib_config_path), 1, calib_config);
    util.verbose_disp(' ', 1, calib_config);
end
