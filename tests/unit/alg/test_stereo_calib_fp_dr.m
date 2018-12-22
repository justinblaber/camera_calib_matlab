function test_stereo_calib_fp_dr
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = util.load_calib_config(fullfile(tests_path, 'data', 'stereo1', 'stereo1.conf'));

    % Set images
    path_cbs_L = {fullfile(tests_path, 'data', 'stereo1', 'left01.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left02.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left03.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left04.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'left05.jpg')};
    path_cbs_R = {fullfile(tests_path, 'data', 'stereo1', 'right01.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right02.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right03.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right04.jpg'), ...
                  fullfile(tests_path, 'data', 'stereo1', 'right05.jpg')};

    % Validate all calibration board images
    img_cbs.L = util.img.validate_similar_imgs(path_cbs_L);
    img_cbs.R = util.img.validate_similar_imgs(path_cbs_R);

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
    calib_test = alg.stereo_calib_fp_dr(img_cbs, ...
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
end
