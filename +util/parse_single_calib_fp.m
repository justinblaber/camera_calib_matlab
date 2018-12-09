function [calib, data] = parse_single_calib_fp(data, suffix)
    % Parses calibration (intrinsics + extrinsics) from input data struct.
    % 
    % Inputs:
    %   data - struct; struct containing calibration
    %   suffix - string; optional suffix appended to names
    %
    % Outputs:
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
    %   data - struct; input data with calibration removed.
                       
    if ~exist('suffix','var')
        suffix = '';
    end
    
    % Read intrinsics ----------------------------------------------------%
    
    % Read A    
    [calib.intrin.A, data] = util.read_and_remove(data, ['A' suffix]);    
    
    % Read distortion
    [calib.intrin.d, data] = util.read_and_remove(data, ['d' suffix]);
    
    % Read extrinsics ----------------------------------------------------%
    
    i = 1;
    while isfield(data, ['img_path_' num2str(i) suffix])
        % Image path
        [calib.extrin(i).img_path, data] = util.read_and_remove(data, ['img_path_' num2str(i) suffix]);
        
        % Rotation
        [calib.extrin(i).R, data] = util.read_and_remove(data, ['R_' num2str(i) suffix]);
        
        % Translation
        [calib.extrin(i).t, data] = util.read_and_remove(data, ['t_' num2str(i) suffix]);

        % Four points in distorted pixel coordinates
        [calib.extrin(i).p_fp_p_ds, data] = util.read_and_remove(data, ['p_fp_p_ds_' num2str(i) suffix]);
        
        % Calibration board points in distorted pixel coordinates
        [calib.extrin(i).p_cb_p_ds, data] = util.read_and_remove(data, ['p_cb_p_ds_' num2str(i) suffix]);
        
        % Covariances of board points in distorted pixel coordinates
        [calib.extrin(i).cov_cb_p_ds, data] = util.read_and_remove(data, ['cov_cb_p_ds_' num2str(i) suffix]);
        
        % Valid calibration board points
        [calib.extrin(i).idx_valid, data] = util.read_and_remove(data, ['idx_valid_' num2str(i) suffix]);
        
        % Increment
        i = i + 1;
    end
end
