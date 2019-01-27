function test_single_calib_fp_dr
    % Checker ------------------------------------------------------------%
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = util.load_calib_config(fullfile(tests_path, 'data', 'checker', 'checker.conf'));

    % Set images
    path_cbs = {fullfile(tests_path, 'data', 'checker', '1.jpg'), ...
                fullfile(tests_path, 'data', 'checker', '2.jpg'), ...
                fullfile(tests_path, 'data', 'checker', '3.jpg')};

    % Validate all calibration board images
    img_cbs = class.img.validate_similar_imgs(path_cbs);

    p_fp_p_dss{1} = [247 312;
                     502 321;
                     257 96;
                     472 113];
    p_fp_p_dss{2} = [292 380;
                     506 372;
                     303 83;
                     507 136];
    p_fp_p_dss{3} = [257 391;
                     474 372;
                     282 126;
                     523 148];

    % Perform single calibration
    calib_test = alg.single_calib_fp_dr(img_cbs, ...
                                        p_fp_p_dss, ...
                                        calib_config);

    % Assert
    load(fullfile(tests_path, 'data', 'checker', 'calib.mat'));
    assert(all(all(abs(calib_test.intrin.A - calib.intrin.A) < 1e-4)));
    assert(all(all(abs(calib_test.intrin.d - calib.intrin.d) < 1e-4)));
    for i = 1:numel(img_cbs)
        assert(all(all(abs(calib_test.extrin(i).R - calib.extrin(i).R) < 1e-4)));
        assert(all(all(abs(calib_test.extrin(i).t - calib.extrin(i).t) < 1e-4)));
    end

    clear

    % Circle -------------------------------------------------------------%
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = util.load_calib_config(fullfile(tests_path, 'data', 'circle', 'circle.conf'));

    % Set images
    path_cbs = {fullfile(tests_path, 'data', 'circle', '1.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '2.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '3.jpg')};

    % Validate all calibration board images
    img_cbs = class.img.validate_similar_imgs(path_cbs);

    p_fp_p_dss{1} = [195 331;
                     505 343;
                     231 86;
                     485 88];
    p_fp_p_dss{2} = [188 359;
                     434 408;
                     200 109;
                     477 139];
    p_fp_p_dss{3} = [276 386;
                     482 364;
                     275 83;
                     475 129];

    % Perform single calibration
    calib_test = alg.single_calib_fp_dr(img_cbs, ...
                                        p_fp_p_dss, ...
                                        calib_config);

    % Assert
    load(fullfile(tests_path, 'data', 'circle', 'calib.mat'));
    assert(all(all(abs(calib_test.intrin.A - calib.intrin.A) < 1e-4)));
    assert(all(all(abs(calib_test.intrin.d - calib.intrin.d) < 1e-4)));
    for i = 1:numel(img_cbs)
        assert(all(all(abs(calib_test.extrin(i).R - calib.extrin(i).R) < 1e-4)));
        assert(all(all(abs(calib_test.extrin(i).t - calib.extrin(i).t) < 1e-4)));
    end
end
