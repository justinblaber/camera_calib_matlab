function dot_vision
    % This example tests out calibration of a stereo camera. It also 
    % demonstrates how to use the automatic four point detection method
    % and also demonstrates how precalibrations can be done on each camera 
    % individually and then the respective intrinsics can be passed into 
    % the stereo calibration so they are not re-optimized. This is useful 
    % because cameras can be pre-calibrated using lots of images, and then
    % stereo calibration can be done later if the user wants to change the 
    % baseline or camera angles.

    % Get test path
    test_path = fileparts(mfilename('fullpath'));

    % Read calibration config
    calib_config = util.read_calib_config(fullfile(test_path,'configs','dot_vision.conf'));

    % Set image path
    img_dir_path = fullfile(test_path,'images','dot_vision');

    % Calibration of left camera
    for i = 1:2
        l_img = dir(fullfile(img_dir_path,['*_' num2str(i) '_L.png']));
        cb_img_paths_L{i} = fullfile(img_dir_path,l_img.name); %#ok<AGROW>
    end

    % Validate all calibration board images
    cb_imgs_L = class.img.validate_similar_imgs(cb_img_paths_L);

    % Get four points
    [four_points_ps_L,four_points_debugs_L] = alg.single_four_points_detect(cb_imgs_L, ...
                                                                            calib_config);

    % Debug four points
    debug.gui_single_four_points_detect(four_points_ps_L, ...
                                        four_points_debugs_L, ...
                                        cb_imgs_L, ...
                                        calib_config);

    % Perform calibration
    calib_L = alg.single_calib_four_points(cb_imgs_L, ...
                                           four_points_ps_L, ...
                                           calib_config);

    % Debug
    debug.gui_single_calib_four_points(calib_L);

    % Calibration of right camera
    for i = 1:2
        l_img = dir(fullfile(img_dir_path,['*_' num2str(i) '_R.png']));
        cb_img_paths_R{i} = fullfile(img_dir_path,l_img.name); %#ok<AGROW>
    end

    % Validate all calibration board images
    cb_imgs_R = class.img.validate_similar_imgs(cb_img_paths_R);

    % Get four points
    [four_points_ps_R,four_points_debugs_R] = alg.single_four_points_detect(cb_imgs_R, ...
                                                                            calib_config);

    % Debug four points
    debug.gui_single_four_points_detect(four_points_ps_R, ...
                                        four_points_debugs_R, ...
                                        cb_imgs_R, ...
                                        calib_config);

    % Perform calibration
    calib_R = alg.single_calib_four_points(cb_imgs_R, ...
                                           four_points_ps_R, ...
                                           calib_config);

    % Debug
    debug.gui_single_calib_four_points(calib_R);

    % Stereo calibration

    for i = 1:2
        % Add 2 to select 3rd and 4th images
        l_img_L = dir(fullfile(img_dir_path,['*_' num2str(i+2) '_L.png']));
        l_img_R = dir(fullfile(img_dir_path,['*_' num2str(i+2) '_R.png']));
        cb_img_paths.L{i} = fullfile(img_dir_path,l_img_L.name);
        cb_img_paths.R{i} = fullfile(img_dir_path,l_img_R.name);
    end

    % Validate all calibration board images
    cb_imgs.L = class.img.validate_similar_imgs(cb_img_paths.L);
    cb_imgs.R = class.img.validate_similar_imgs(cb_img_paths.R);

    % Get four points
    [four_points_ps,four_points_debugs] = alg.stereo_four_points_detect(cb_imgs, ...
                                                                        calib_config);

    % Debug four points
    debug.gui_stereo_four_points_detect(four_points_ps, ...
                                        four_points_debugs, ...
                                        cb_imgs, ...
                                        calib_config);

    % Perform calibration of extrinsics only
    intrin.L = calib_L.intrin;
    intrin.R = calib_R.intrin;
    [calib,R_s,t_s] = alg.stereo_calib_four_points(cb_imgs, ...
                                                   four_points_ps, ...
                                                   calib_config, ...
                                                   intrin);

    % Debug
    debug.gui_stereo_calib_four_points(calib,R_s,t_s);

    % Perform full calibration
    [calib,R_s,t_s] = alg.stereo_calib_four_points(cb_imgs, ...
                                                   four_points_ps, ...
                                                   calib_config);

    % Debug
    debug.gui_stereo_calib_four_points(calib,R_s,t_s);
end
