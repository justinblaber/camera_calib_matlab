function single1
    % This example tests out calibration of a single camera by using the "four
    % point method" which is similar to Bouguet's toolbox.

    % Get test path and name
    [test_path, test_name] = fileparts(mfilename('fullpath'));

    % Read calibration config
    calib_config = util.read_calib_config(fullfile(test_path,'configs','single.conf'));

    % Set images
    img_dir_path = fullfile(test_path,'images','single');
    num_imgs = 8;
    for i = 1:num_imgs
        cb_img_paths{i} = fullfile(img_dir_path,['Image' num2str(i) '.tif']); %#ok<AGROW>
    end     

    % Validate all calibration board images
    cb_imgs = class.img.validate_similar_imgs(cb_img_paths);

    % Get four points in pixel coordinates per calibration board image
    four_points_ps{1} = [168.7272  179.7808
                         133.9287  377.5377
                         413.8812  194.8252
                         472.3155  386.7228];

    four_points_ps{2} = [172.3350  108.0685
                         138.0881  379.0046
                         444.4066  129.1560
                         481.5573  389.7946];

    four_points_ps{3} = [199.7681  98.1806
                         117.8841  354.4820
                         472.0018  142.9956
                         460.8990  413.7783]; 

    four_points_ps{4} = [250.2379  114.2989
                         145.3761  361.4282
                         534.0855  89.3777
                         509.4070  393.5712]; 

    four_points_ps{5} = [82.6358   195.8193
                         403.7691  430.2562
                         223.6562  44.6192
                         432.8029  218.4991]; 

    four_points_ps{6} = [91.4560   129.4162
                         117.4309  398.5697
                         535.8790  172.1372
                         423.0276  413.6498]; 

    four_points_ps{7} = [181.9291  128.9180
                         165.0031  453.1418
                         481.8723  96.0359
                         406.0600  351.5853]; 

    four_points_ps{8} = [72.2896   102.1230
                         88.7938   426.2946
                         371.3808  66.0314
                         325.0443  322.2007]; 

    % Perform single calibration
    calib = alg.single_calib_four_points(cb_imgs, ...
                                         four_points_ps, ...
                                         calib_config);

    % Save calibration
    calib_path = fullfile(test_path,'calibrations',[test_name '.txt']);
    util.write_single_calib_four_points(calib,calib_path);

    % Read calibration
    clearvars -except calib_path;

    calib = util.read_single_calib_four_points(calib_path);

    % Debug with gui
    debug.gui_single_calib_four_points(calib);
end