function save_multi_calib(calib, file_path)

    % This will clear the file
    fclose(fopen(file_path, 'w'));

    % Write config
    util.write_comment('Calibration configuration', file_path);
    util.write_data(calib.config, file_path);
    util.write_newline(file_path);

    % Write each cameras calibration
    for i = 1:numel(calib.cam)
        util.write_single_calib(calib.cam(i), file_path, ['_cam' num2str(i)]);
    end
end
