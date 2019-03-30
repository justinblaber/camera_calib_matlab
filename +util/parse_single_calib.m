function [calib, data] = parse_single_calib(data, suffix)

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
    while isfield(data, ['obj_img' num2str(i) suffix])
        % Calibration board image
        [obj_img, data] = util.read_and_remove(data, ['obj_img' num2str(i) suffix]);                 % Read class
        [state_img, data] = util.read_and_remove(data, ['state_img' num2str(i) suffix]); %#ok<ASGLU> % Read state
        calib.extrin(i).img_cb = eval([obj_img '(state_img);']);                                     % Instantiate

        % TODO: maybe issue warning if image cant be found?

        % Rotation
        [calib.extrin(i).R, data] = util.read_and_remove(data, ['R' num2str(i) suffix]);

        % Translation
        [calib.extrin(i).t, data] = util.read_and_remove(data, ['t' num2str(i) suffix]);

        % Calibration board distorted pixel points
        [calib.extrin(i).p_cb_p_ds, data] = util.read_and_remove(data, ['p_cb_p_ds' num2str(i) suffix]);

        % Covariances of calibration board distorted pixel points
        [calib.extrin(i).cov_cb_p_ds, data] = util.read_and_remove(data, ['cov_cb_p_ds' num2str(i) suffix]);

        % Calibration board model distorted pixel points
        [calib.extrin(i).p_cb_p_d_ms, data] = util.read_and_remove(data, ['p_cb_p_d_ms' num2str(i) suffix]);

        % Valid calibration board points - convert to logical
        [calib.extrin(i).idx_valid, data] = util.read_and_remove(data, ['idx_valid' num2str(i) suffix]);
        calib.extrin(i).idx_valid = logical(calib.extrin(i).idx_valid);

        % Four point box around the calibration board image in distorted pixel coordinates
        if isfield(data, ['p_fp_p_ds' num2str(i) suffix])
            [calib.extrin(i).p_fp_p_ds, data] = util.read_and_remove(data, ['p_fp_p_ds' num2str(i) suffix]);
        end

        % Increment
        i = i + 1;
    end

    % R_1
    [calib.R_1, data] = util.read_and_remove(data, ['R_1' suffix]);

    % t_1
    [calib.t_1, data] = util.read_and_remove(data, ['t_1' suffix]);
end
