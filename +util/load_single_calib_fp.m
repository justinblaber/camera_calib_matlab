function [calib, calib_config] = load_single_calib_fp(file_path)
    % Reads a four point single calibration from file path
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
    %           .img_path - string; path to calibration board image
    %           .R - array; 3x3 rotation matrix
    %           .t - array; 3x1 translation vector
    %           .p_fp_p_ds - array; four point box around the
    %               calibration board image in distorted pixel coordinates
    %           .p_cb_p_ds - array; calibration board points in distorted
    %               pixel coordinates
    %           .cov_cb_p_ds - cell; covariances of calibration board
    %               points in distorted pixel coordinates
    %           .idx_valid - array; valid calibration board points
    %   calib_config - struct; struct returned by util.read_calib_config()

    % Check to make sure data file exists
    if exist(file_path, 'file') ~= 2
        error(['Calibration file: ' file_path ' does not exist.']);
    end

    % Read data
    data = util.read_data(file_path);

    % Parse out single four point calibration
    [calib, data] = util.parse_single_calib_fp(data);
    
    % Parse out calib config
    [calib_config, data] = util.parse_calib_config(data);

    % Display error if there are still fields existing in data
    fields_data = fields(data);
    if numel(fields_data) ~= 0
        error(['When reading calibration, the following unknown ' ...
               'fields were found: "' strjoin(fields_data, ', ') '".']);
    end
end
