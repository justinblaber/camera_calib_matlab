function [calib, data] = parse_single_calib(data, suffix)
    % Parses calibration (intrinsics + extrinsics) from input data struct.
    %
    % Inputs:
    %   data - struct; struct containing calibration
    %   suffix - string; optional. suffix appended to names
    %
    % Outputs:
    %   calib - struct;
    %       .intrin - struct;
    %           .A - array; 3x3 camera matrix
    %           .d - array; Mx1 array of distortion coefficients
    %       .extrin - struct; Nx1 struct containing extrinsics
    %           .img_cb - class.img.base; calibration board image
    %           .R - array; 3x3 rotation matrix
    %           .t - array; 3x1 translation vector
    %           .p_fp_p_ds - array; optional. four point box around the
    %               calibration board image in distorted pixel coordinates
    %           .p_cb_p_ds - array; calibration board distorted pixel
    %               points
    %           .cov_cb_p_ds - cell; covariances of calibration board
    %               distorted pixel points
    %           .p_cb_p_d_ms - array; calibration board model distorted
    %               pixel points
    %           .idx_valid - array; valid calibration board points
    %   data - struct; input data with calibration removed.

    % Get suffix
    if ~exist('suffix', 'var')
        suffix = '';
    end

    % Read intrinsics ----------------------------------------------------%

    % Camera matrix
    [calib.intrin.A, data] = util.read_and_remove(data, ['A' suffix]);

    % Distortion coefficients
    [calib.intrin.d, data] = util.read_and_remove(data, ['d' suffix]);

    % Read extrinsics ----------------------------------------------------%

    i = 1;
    while isfield(data, ['img_path_' num2str(i) suffix])
        % Calibration board image - read path and then convert to
        % class.img.base
        [calib.extrin(i).img_cb, data] = util.read_and_remove(data, ['img_path_' num2str(i) suffix]);
        calib.extrin(i).img_cb = class.img(calib.extrin(i).img_cb);

        % TODO: maybe issue warning if image cant be found?

        % Rotation
        [calib.extrin(i).R, data] = util.read_and_remove(data, ['R_' num2str(i) suffix]);

        % Translation
        [calib.extrin(i).t, data] = util.read_and_remove(data, ['t_' num2str(i) suffix]);

        % Four point box around the calibration board image in distorted pixel coordinates
        if isfield(data, ['p_fp_p_ds_' num2str(i) suffix])
            [calib.extrin(i).p_fp_p_ds, data] = util.read_and_remove(data, ['p_fp_p_ds_' num2str(i) suffix]);
        end

        % Calibration board distorted pixel points
        [calib.extrin(i).p_cb_p_ds, data] = util.read_and_remove(data, ['p_cb_p_ds_' num2str(i) suffix]);

        % Covariances of calibration board distorted pixel points
        [calib.extrin(i).cov_cb_p_ds, data] = util.read_and_remove(data, ['cov_cb_p_ds_' num2str(i) suffix]);

        % Calibration board model distorted pixel points
        [calib.extrin(i).p_cb_p_d_ms, data] = util.read_and_remove(data, ['p_cb_p_d_ms_' num2str(i) suffix]);

        % Valid calibration board points - convert to logical
        [calib.extrin(i).idx_valid, data] = util.read_and_remove(data, ['idx_valid_' num2str(i) suffix]);
        calib.extrin(i).idx_valid = logical(calib.extrin(i).idx_valid);

        % Increment
        i = i + 1;
    end
end
