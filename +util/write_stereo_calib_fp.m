function write_stereo_calib_fp(calib, file_path)
    % Writes outputs of four point stereo calibration to a file, so it can
    % be read again later.
    %
    % Inputs:
    %   calib - struct;
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

    % Write left
    util.write_single_calib_fp(calib.L, file_path, '_L');

    % Write right
    util.write_single_calib_fp(calib.R, file_path, '_R', true);

    % Write R_s and t_s
    util.write_array(calib.R_s, 'R_s', file_path);
    util.write_newline(file_path);
    util.write_array(calib.t_s, 't_s', file_path);
    util.write_newline(file_path);
end
