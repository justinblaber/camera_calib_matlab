function write_single_cam(cam, file_path, suffix)
    % Writes a single camera calibration to file.
    %
    % Inputs:
    %   cam - struct;
    %       .intrin - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %       .extrin - struct; Nx1 struct containing extrinsics
    %           .img_cb - class.img.intf; calibration board image
    %           .R - array; 3x3 rotation matrix
    %           .t - array; 3x1 translation vector
    %           .p_cb_p_ds - array; calibration board distorted pixel
    %               points
    %           .cov_cb_p_ds - cell; covariances of calibration board
    %               distorted pixel points
    %           .p_cb_p_d_ms - array; calibration board model distorted
    %               pixel points
    %           .idx_valid - array; valid calibration board points
    %           .p_fp_p_ds - array; optional. four point box around the
    %               calibration board image in distorted pixel coordinates
    %       .R_1 - array; relative rotation between camera "1" and this
    %           camera
    %       .t_1 - array; relative translation between camera "1" and
    %           this camera
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
    util.write_array(cam.intrin.A, ['A' suffix], file_path);
    util.write_newline(file_path);

    % Distortion coefficients
    util.write_array(cam.intrin.d, ['d' suffix], file_path);
    util.write_newline(file_path);

    % Write extrinsics ---------------------------------------------------%

    for i = 1:numel(cam.extrin)
        util.write_comment(['Extrinsics' num2str(i) suffix], file_path);

        % Calibration board image
        util.write_string(class(cam.extrin(i).img_cb), ['obj_img' num2str(i) suffix], file_path);  % Write class
        cam.extrin(i).img_cb.write(['state_img' num2str(i) suffix], file_path);                    % Write state
        util.write_newline(file_path);

        % Rotation
        util.write_array(cam.extrin(i).R, ['R' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Translation
        util.write_array(cam.extrin(i).t, ['t' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Calibration board distorted pixel points
        util.write_array(cam.extrin(i).p_cb_p_ds, ['p_cb_p_ds' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Covariances of calibration board distorted pixel points
        for j = 1:numel(cam.extrin(i).cov_cb_p_ds)
            util.write_array(cam.extrin(i).cov_cb_p_ds{j}, ['cov_cb_p_ds' num2str(i) suffix], file_path);
        end
        util.write_newline(file_path);

        % Calibration board model distorted pixel points
        util.write_array(cam.extrin(i).p_cb_p_d_ms, ['p_cb_p_d_ms' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Valid calibration board points
        util.write_array(cam.extrin(i).idx_valid, ['idx_valid' num2str(i) suffix], file_path);
        util.write_newline(file_path);

        % Four point box around the calibration board image in distorted pixel coordinates
        if isfield(cam.extrin(i), 'p_fp_p_ds')
            util.write_array(cam.extrin(i).p_fp_p_ds, ['p_fp_p_ds' num2str(i) suffix], file_path);
            util.write_newline(file_path);
        end
    end

    % Write R_1
    util.write_array(cam.R_1, ['R_1' suffix], file_path);
    util.write_newline(file_path);

    % Write t_1
    util.write_array(cam.t_1, ['t_1' suffix], file_path);
    util.write_newline(file_path);
end
