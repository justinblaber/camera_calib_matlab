function calib = load_single_calib_fp(file_path)
    % Loads a four point single calibration from file path
    %
    % Inputs:
    %   file_path - string; path to calibration
    %
    % Outputs:
    %   calib - struct;
    %       .config - struct; calibration config
    %       .intrin - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %       .extrin - struct; Nx1 struct containing extrinsics
    %           .img_cb - util.img; calibration board image
    %           .R - array; 3x3 rotation matrix
    %           .t - array; 3x1 translation vector
    %           .p_fp_p_ds - array; four point box around the calibration
    %               board image in distorted pixel coordinates
    %           .p_cb_p_ds - array; calibration board distorted pixel
    %               points
    %           .cov_cb_p_ds - cell; covariances of calibration board
    %               distorted pixel points
    %           .p_cb_p_d_ms - array; calibration board model distorted
    %               pixel points
    %           .idx_valid - array; valid calibration board points

    % Check to make sure data file exists
    if exist(file_path, 'file') ~= 2
        error(['Calibration file: ' file_path ' does not exist.']);
    end

    % Read data
    data = util.read_data(file_path);

    % Parse out single four point calibration
    [calib_tmp, data] = util.parse_single_calib_fp(data);

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
    calib.intrin = calib_tmp.intrin;
    calib.extrin = calib_tmp.extrin;
end
