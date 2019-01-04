function write_single_calib_fp(calib, file_path, suffix)
    % Writes a single four point calibration to file.
    %
    % Inputs:
    %   calib - struct;
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
    %   file_path - string; path to calibration
    %   suffix - string; optional. suffix to add to names
    %
    % Outputs:
    %   None

    % Get suffix
    if ~exist('suffix', 'var')
        suffix = '';
    end

    % Write intrinsics ---------------------------------------------------%

    util.write_comment(['Intrinsics' suffix], file_path);

    % Camera matrix
    util.write_array(calib.intrin.A, ['A' suffix], file_path);
    util.write_newline(file_path);

    % Distortion coefficients
    util.write_array(calib.intrin.d, ['d' suffix], file_path);
    util.write_newline(file_path);

    % Write extrinsics ---------------------------------------------------%

    for i = 1:numel(calib.extrin)
        util.write_comment(['Extrinsics_' num2str(i) suffix], file_path);

        % Calibration board image - write path
        util.write_string(calib.extrin(i).img_cb.get_path(), ['img_path_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Rotation
        util.write_array(calib.extrin(i).R, ['R_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Translation
        util.write_array(calib.extrin(i).t, ['t_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Four point box around the calibration board image in distorted pixel coordinates
        util.write_array(calib.extrin(i).p_fp_p_ds, ['p_fp_p_ds_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Calibration board distorted pixel points
        util.write_array(calib.extrin(i).p_cb_p_ds, ['p_cb_p_ds_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Covariances of calibration board distorted pixel points
        for j = 1:numel(calib.extrin(i).cov_cb_p_ds)
            util.write_array(calib.extrin(i).cov_cb_p_ds{j}, ['cov_cb_p_ds_' num2str(i) suffix], file_path);
        end
        util.write_newline(file_path);
        
        % Calibration board model distorted pixel points
        util.write_array(calib.extrin(i).p_cb_p_d_ms, ['p_cb_p_d_ms_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Valid calibration board points
        util.write_array(calib.extrin(i).idx_valid, ['idx_valid_' num2str(i) suffix], file_path);
        util.write_newline(file_path);
    end
end
