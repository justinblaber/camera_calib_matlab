function write_single_calib_fp(calib,file_path,suffix,append_calib)
    
    % Get suffix
    if ~exist('suffix','var')
        suffix = '';
    end
    
    % Write calib config -------------------------------------------------%
    
    if ~exist('append_calib','var') || ~append_calib
        % This will clear the file
        fclose(fopen(file_path,'w'));

        % Write calib_config
        util.write_comment('Calibration configuration', file_path);
        util.write_data(calib.config, file_path);
        util.write_newline(file_path);
    end
    
    % Write intrinsics ---------------------------------------------------%
    
    util.write_comment(['Intrinsic parameters ' suffix], file_path);
            
    % Write A    
    util.write_array(calib.intrin.A,['A' suffix],file_path);
    util.write_newline(file_path);
    
    % Write distortion
    util.write_array(calib.intrin.d,['d' suffix],file_path);
    util.write_newline(file_path);
    
    % Write extrinsics ---------------------------------------------------%
    
    for i = 1:numel(calib.extrin)
        util.write_comment(['Extrinsic parameters for image: ' calib.extrin(i).img_path ''], file_path);
        
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
