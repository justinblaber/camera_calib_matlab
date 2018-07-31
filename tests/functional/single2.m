function single2
    % This example tests out calibration of a single camera by using the "four
    % point method" which is similar to Bouguet's toolbox.

    % Get test path and name
    [test_path, test_name] = fileparts(mfilename('fullpath'));

    % Read calibration config
    calib_config = util.read_calib_config(fullfile(test_path,'configs','stereo.conf'));

    % Set images
    img_dir_path = fullfile(test_path,'images','stereo');
    num_imgs = 5;
    for i = 1:num_imgs
        cb_img_paths{i} = fullfile(img_dir_path,['left0' num2str(i) '.jpg']); %#ok<AGROW>
    end     

    % Validate all calibration board images
    cb_imgs = class.img.validate_similar_imgs(cb_img_paths);

    % Get four points in pixel coordinates per calibration board image
    four_points_ps{1} = [245.4038  95.1070
                         249.9282  254.6160
                         478.6004  87.1966
                         476.3041  265.6548];

    four_points_ps{2} = [252.0843  128.9690
                         524.5152  182.0328
                         257.1694  358.2938
                         438.8563  397.7544];

    four_points_ps{3} = [278.1340  73.1726
                         188.3629  258.4665
                         563.2866  154.6281
                         498.8266  375.6217];

    four_points_ps{4} = [189.5038  131.6406
                         180.3775  329.1699
                         471.2737  111.2002
                         476.1010  339.2265]; 

    four_points_ps{5} = [241.9079  98.0027
                         437.3147  50.8201
                         280.5987  379.7500
                         543.3684  314.8763];

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
