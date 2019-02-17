function save_stereo_calib(calib, file_path)
    % Saves a stereo calibration to file.
    %
    % Inputs:
    %   calib - struct;
    %       .config - struct; calibration config
    %       .L - struct; calibration for left camera
    %       .R - struct; calibration for right camera
    %       .R_s - array; 3x3 rotation matrix describing rotation from
    %           left to right camera
    %       .t_s - array; 3x1 translation vector describing translation
    %           from left to right camera
    %   file_path - string; path to calibration
    %
    % Outputs:
    %   None

    % This will clear the file
    fclose(fopen(file_path, 'w'));

    % Write config
    util.write_comment('Calibration configuration', file_path);
    util.write_data(calib.config, file_path);
    util.write_newline(file_path);

    % Write left
    util.write_single_calib(calib.L, file_path, '_L');

    % Write right
    util.write_single_calib(calib.R, file_path, '_R');

    % Write R_s and t_s
    util.write_array(calib.R_s, 'R_s', file_path);
    util.write_newline(file_path);
    util.write_array(calib.t_s, 't_s', file_path);
    util.write_newline(file_path);
end
