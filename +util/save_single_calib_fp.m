function save_single_calib_fp(calib, calib_config, file_path)
    % Saves a single four point calibration to file.
    %
    % Inputs:
    %   calib - struct;
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
    %   file_path - string; path to calibration
    %   suffix - string; optional. suffix to add to names
    %
    % Outputs:
    %   None
        
    % Clear file
    fclose(fopen(file_path, 'w'));

    % Write calib_config
    util.write_comment('Calibration configuration', file_path);
    util.write_data(calib_config, file_path);
    util.write_newline(file_path);
    
    % Write calib
    util.write_single_calib_fp(calib, file_path);
end
