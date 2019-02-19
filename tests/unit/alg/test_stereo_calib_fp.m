function test_stereo_calib_fp
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = intf.load_calib_config(fullfile(tests_path, 'data', 'stereo1', 'stereo1.conf'));

    % Set images
    path_cbs.L = {fullfile(tests_path, 'data', 'stereo1', 'left01.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left02.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left03.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left04.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left05.jpg')};
    path_cbs.R = {fullfile(tests_path, 'data', 'stereo1', 'right01.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right02.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right03.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right04.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right05.jpg')};

    % Validate all calibration board images
    img_cbs = intf.validate_stereo_imgs(path_cbs);

    % Get four points in pixel coordinates per calibration board image
    p_fp_p_dss.L{1} = [275 91
                       279 255
                       479 86
                       476 264];
    p_fp_p_dss.R{1} = [155 107
                       161 266
                       344 93
                       347 278];

    p_fp_p_dss.L{2} = [255 335
                       447 376
                       252 128
                       523 180];
    p_fp_p_dss.R{2} = [121 344
                       302 390
                       71 148
                       324 191];

    p_fp_p_dss.L{3} = [315 80
                       227 273
                       564 154
                       498 375];
    p_fp_p_dss.R{3} = [165 97
                       71 282
                       403 160
                       314 391];

    p_fp_p_dss.L{4} = [225 127
                       216 330
                       471 109
                       476 338];
    p_fp_p_dss.R{4} = [87 145
                       75 339
                       309 119
                       306 353];

    p_fp_p_dss.L{5} = [244 126
                       450 78
                       279 378
                       542 313];
    p_fp_p_dss.R{5} = [100 144
                       295 87
                       99 385
                       352 330];

    % Perform stereo calibration
    calib_test = alg.stereo_calib_fp(@alg.single_calib_fp_dr, ...
                                     img_cbs, ...
                                     p_fp_p_dss, ...
                                     calib_config);

    % Assert
    load(fullfile(tests_path, 'data', 'stereo1', 'calib.mat'));
    assert(all(all(abs(calib_test.L.intrin.A - calib.L.intrin.A) < 1e-4)));
    assert(all(all(abs(calib_test.L.intrin.d - calib.L.intrin.d) < 1e-4)));
    assert(all(all(abs(calib_test.R.intrin.A - calib.R.intrin.A) < 1e-4)));
    assert(all(all(abs(calib_test.R.intrin.d - calib.R.intrin.d) < 1e-4)));
    for i = 1:numel(img_cbs)
        assert(all(all(abs(calib_test.L.extrin(i).R - calib.L.extrin(i).R) < 1e-4)));
        assert(all(all(abs(calib_test.L.extrin(i).t - calib.L.extrin(i).t) < 1e-4)));
        assert(all(all(abs(calib_test.R.extrin(i).R - calib.R.extrin(i).R) < 1e-4)));
        assert(all(all(abs(calib_test.R.extrin(i).t - calib.R.extrin(i).t) < 1e-4)));
    end
    
    clear
    
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = intf.load_calib_config(fullfile(tests_path, 'data', 'stereo2', 'stereo2.conf'));

    % Set images
    path_cbs.L = {fullfile(tests_path, 'data', 'stereo2', 'IMG_1_L.JPG')};
    path_cbs.R = {fullfile(tests_path, 'data', 'stereo2', 'IMG_1_R.JPG')};

    % Validate all calibration board images
    img_cbs = intf.validate_stereo_imgs(path_cbs);

    % Get four points in pixel coordinates per calibration board image
    p_fp_p_dss.L{1} = 1.0e+02 * [5.823333333333332   4.450000000000000
                                 5.666666666666666   2.576666666666666
                                 3.750000000000000   5.086666666666666
                                 3.853333333333333   3.273333333333333];
    p_fp_p_dss.R{1} = 1.0e+02 * [6.686666666666667   5.616666666666666
                                 6.466666666666666   3.170000000000000
                                 4.356666666666666   5.606666666666666
                                 4.270000000000000   2.783333333333333];

    % Perform stereo calibration
    calib_test = alg.stereo_calib_fp(@alg.single_calib_fp_dr, ...
                                     img_cbs, ...
                                     p_fp_p_dss, ...
                                     calib_config);

    % Assert
    load(fullfile(tests_path, 'data', 'stereo2', 'calib.mat'));
    assert(all(all(abs(calib_test.L.intrin.A - calib.L.intrin.A) < 1e-4)));
    assert(all(all(abs(calib_test.L.intrin.d - calib.L.intrin.d) < 1e-4)));
    assert(all(all(abs(calib_test.R.intrin.A - calib.R.intrin.A) < 1e-4)));
    assert(all(all(abs(calib_test.R.intrin.d - calib.R.intrin.d) < 1e-4)));
    for i = 1:numel(img_cbs)
        assert(all(all(abs(calib_test.L.extrin(i).R - calib.L.extrin(i).R) < 1e-4)));
        assert(all(all(abs(calib_test.L.extrin(i).t - calib.L.extrin(i).t) < 1e-4)));
        assert(all(all(abs(calib_test.R.extrin(i).R - calib.R.extrin(i).R) < 1e-4)));
        assert(all(all(abs(calib_test.R.extrin(i).t - calib.R.extrin(i).t) < 1e-4)));
    end
end
