function test_single_calib_H_dr
    % Checker ------------------------------------------------------------%
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = intf.load_calib_config(fullfile(tests_path, 'data', 'checker', 'checker.conf'));

    % Set images
    path_cbs = {fullfile(tests_path, 'data', 'checker', '1.jpg'), ...
                fullfile(tests_path, 'data', 'checker', '2.jpg'), ...
                fullfile(tests_path, 'data', 'checker', '3.jpg')};

    % Validate all calibration board images
    img_cbs = intf.validate_single_imgs(path_cbs);

    H_w2ps{1} = 1.0e+02 * [ 0.001304184447162   0.006215160306952  -0.158506044500098
                           -0.004380249891446   0.000566936427656   5.454688718406464
                            0.000004190697299   0.000001112717747   0.010000000000000];
    H_w2ps{2} = 1.0e+02 * [ 0.000600494733427   0.008211722053490   0.088444423457931
                           -0.006930003020493   0.002120026733941   7.025200331297144
                            0.000001168792830   0.000006130905887   0.010000000000000];
    H_w2ps{3} = 1.0e+02 * [-0.000197258369069   0.005571801863219   0.369792935389606
                           -0.005264814064386   0.000813214136621   5.935510362600152
                           -0.000002387039235   0.000003143417853   0.010000000000000];

    % Perform single calibration
    calib_test = alg.single_calib_H_dr(calib_config.obj_A, ...
                                       calib_config.obj_R, ...
                                       class.calib.cb_w2p_p2p(calib_config), ...
                                       class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config), ...
                                       calib_config.obj_cb_geom, ...
                                       img_cbs, ...
                                       H_w2ps, ...
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
    calib_config = intf.load_calib_config(fullfile(tests_path, 'data', 'circle', 'circle.conf'));

    % Set images
    path_cbs = {fullfile(tests_path, 'data', 'circle', '1.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '2.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '3.jpg')};

    % Validate all calibration board images
    img_cbs = intf.validate_single_imgs(path_cbs);

    H_w2ps{1} = 1.0e+02 * [ 0.001802256209152   0.006214020050158  -1.126612165558416
                           -0.004809098151523   0.000037084950548   5.896016709922130
                            0.000004518634317  -0.000000646784419   0.010000000000000];
    H_w2ps{2} = 1.0e+02 * [-0.000139293574909   0.003341440269200   0.291741159053135
                           -0.004154608742963   0.000241634973300   4.817696702032065
                           -0.000001595751975  -0.000001316615653   0.010000000000000];
    H_w2ps{3} = 1.0e+02 * [ 0.000411757065057   0.008855051250076  -0.132033543324934
                           -0.007475581537464   0.002192787577082   7.605609328773577
                            0.000001425460400   0.000007559557856   0.010000000000000];

    % Perform single calibration
    calib_test = alg.single_calib_H_dr(calib_config.obj_A, ...
                                       calib_config.obj_R, ...
                                       class.calib.cb_w2p_c2e(calib_config), ...
                                       class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config), ...
                                       calib_config.obj_cb_geom, ...
                                       img_cbs, ...
                                       H_w2ps, ...
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
