function write_single_calib_fp(calib, file_path, suffix)
    % Writes a single four point calibration to file.
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

    % Write A
    util.write_array(calib.intrin.A, ['A' suffix], file_path);
    util.write_newline(file_path);

    % Write distortion
    util.write_array(calib.intrin.d, ['d' suffix], file_path);
    util.write_newline(file_path);

    % Write extrinsics ---------------------------------------------------%

    for i = 1:numel(calib.extrin)
        util.write_comment(['Extrinsics_' num2str(i) suffix], file_path);

        % Image path
        util.write_string(calib.extrin(i).img_path, ['img_path_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Rotation
        util.write_array(calib.extrin(i).R, ['R_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Translation
        util.write_array(calib.extrin(i).t, ['t_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Four points in distorted pixel coordinates
        util.write_array(calib.extrin(i).p_fp_p_ds, ['p_fp_p_ds_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Calibration board points in distorted pixel coordinates
        util.write_array(calib.extrin(i).p_cb_p_ds, ['p_cb_p_ds_' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Covariances of board points in distorted pixel coordinates
        for j = 1:numel(calib.extrin(i).cov_cb_p_ds)
            util.write_array(calib.extrin(i).cov_cb_p_ds{j}, ['cov_cb_p_ds_' num2str(i) suffix], file_path);
        end
        util.write_newline(file_path);

        % Valid calibration board points
        util.write_array(calib.extrin(i).idx_valid, ['idx_valid_' num2str(i) suffix], file_path);
        util.write_newline(file_path);
    end
end
