function [cam, data] = parse_single_cam(data, suffix)
    % Parses a single camera calibration from input data struct.
    %
    % Inputs:
    %   data - struct; struct containing single camera calibration
    %   suffix - string; optional. suffix appended to names
    %
    % Outputs:
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
    %   data - struct; input data with single camera calibration removed.

    % Get suffix
    if ~exist('suffix', 'var')
        suffix = '';
    end

    % Read intrinsics ----------------------------------------------------%

    % Camera matrix
    [cam.intrin.A, data] = util.read_and_remove(data, ['A' suffix]);

    % Distortion coefficients
    [cam.intrin.d, data] = util.read_and_remove(data, ['d' suffix]);

    % Read extrinsics ----------------------------------------------------%

    i = 1;
    while isfield(data, ['obj_img' num2str(i) suffix])
        % Calibration board image
        [obj_img, data] = util.read_and_remove(data, ['obj_img' num2str(i) suffix]);                 % Read class
        [state_img, data] = util.read_and_remove(data, ['state_img' num2str(i) suffix]); %#ok<ASGLU> % Read state
        cam.extrin(i).img_cb = eval([obj_img '(state_img);']);                                       % Instantiate

        % Rotation
        [cam.extrin(i).R, data] = util.read_and_remove(data, ['R' num2str(i) suffix]);

        % Translation
        [cam.extrin(i).t, data] = util.read_and_remove(data, ['t' num2str(i) suffix]);

        % Calibration board distorted pixel points
        [cam.extrin(i).p_cb_p_ds, data] = util.read_and_remove(data, ['p_cb_p_ds' num2str(i) suffix]);

        % Covariances of calibration board distorted pixel points
        [cam.extrin(i).cov_cb_p_ds, data] = util.read_and_remove(data, ['cov_cb_p_ds' num2str(i) suffix]);

        % Calibration board model distorted pixel points
        [cam.extrin(i).p_cb_p_d_ms, data] = util.read_and_remove(data, ['p_cb_p_d_ms' num2str(i) suffix]);

        % Valid calibration board points - convert to logical
        [cam.extrin(i).idx_valid, data] = util.read_and_remove(data, ['idx_valid' num2str(i) suffix]);
        cam.extrin(i).idx_valid = logical(cam.extrin(i).idx_valid);

        % Four point box around the calibration board image in distorted pixel coordinates
        if isfield(data, ['p_fp_p_ds' num2str(i) suffix])
            [cam.extrin(i).p_fp_p_ds, data] = util.read_and_remove(data, ['p_fp_p_ds' num2str(i) suffix]);
        end

        % Increment
        i = i + 1;
    end

    % R_1
    [cam.R_1, data] = util.read_and_remove(data, ['R_1' suffix]);

    % t_1
    [cam.t_1, data] = util.read_and_remove(data, ['t_1' suffix]);
end
