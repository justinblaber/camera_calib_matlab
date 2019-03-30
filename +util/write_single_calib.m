function write_single_calib(calib, file_path, suffix)

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
        util.write_comment(['Extrinsics' num2str(i) suffix], file_path);

        % Calibration board image
        util.write_string(class(calib.extrin(i).img_cb), ['obj_img' num2str(i) suffix], file_path);  % Write class
        calib.extrin(i).img_cb.write(['state_img' num2str(i) suffix], file_path);                    % Write state
        util.write_newline(file_path);

        % Rotation
        util.write_array(calib.extrin(i).R, ['R' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Translation
        util.write_array(calib.extrin(i).t, ['t' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Calibration board distorted pixel points
        util.write_array(calib.extrin(i).p_cb_p_ds, ['p_cb_p_ds' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Covariances of calibration board distorted pixel points
        for j = 1:numel(calib.extrin(i).cov_cb_p_ds)
            util.write_array(calib.extrin(i).cov_cb_p_ds{j}, ['cov_cb_p_ds' num2str(i) suffix], file_path);
        end
        util.write_newline(file_path);

        % Calibration board model distorted pixel points
        util.write_array(calib.extrin(i).p_cb_p_d_ms, ['p_cb_p_d_ms' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Valid calibration board points
        util.write_array(calib.extrin(i).idx_valid, ['idx_valid' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Four point box around the calibration board image in distorted pixel coordinates
        if isfield(calib.extrin(i), 'p_fp_p_ds')
            util.write_array(calib.extrin(i).p_fp_p_ds, ['p_fp_p_ds' num2str(i) suffix], file_path);
            util.write_newline(file_path);
        end
    end

    % Write R_1
    util.write_array(calib.R_1, ['R_1' suffix], file_path);
    util.write_newline(file_path);

    % Write t_1
    util.write_array(calib.t_1, ['t_1' suffix], file_path);
    util.write_newline(file_path);
end
