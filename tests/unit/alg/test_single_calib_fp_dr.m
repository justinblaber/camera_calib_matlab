function test_single_calib_fp_dr
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

    p_fp_p_dss{1} = 1.0e+02 * [2.478560080855610   3.128811574943620;
                               5.017550646217204   3.222340184937016;
                               2.578858092193191   0.978713657276637;
                               4.728057229926550   1.144768037695122];
    p_fp_p_dss{2} = 1.0e+02 * [2.925910339834197   3.801571522076829;
                               5.068529837395690   3.731235271114984;
                               3.029352962698060   0.850811968254622;
                               5.071128818731172   1.364401069831986];
    p_fp_p_dss{3} = 1.0e+02 * [2.571316156472532   3.917643468669752;
                               4.745479876584502   3.726739573724457;
                               2.825616445648377   1.274264610508025;
                               5.224669577204050   1.485639129893151];

    % Perform single calibration
    calib_test = alg.single_calib_fp_dr(calib_config.obj_A, ...
                                        calib_config.obj_R, ...
                                        class.calib.cb_w2p_p2p(calib_config), ...
                                        class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config), ...
                                        calib_config.obj_cb_geom, ...
                                        img_cbs, ...
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
    calib_config = intf.load_calib_config(fullfile(tests_path, 'data', 'circle', 'circle.conf'));

    % Set images
    path_cbs = {fullfile(tests_path, 'data', 'circle', '1.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '2.jpg'), ...
                fullfile(tests_path, 'data', 'circle', '3.jpg')};

    % Validate all calibration board images
    img_cbs = intf.validate_single_imgs(path_cbs);

    p_fp_p_dss{1} = 1.0e+02 * [1.958093817366979   3.321367973803599
                               5.047045883488412   3.444087740490789
                               2.315190657038523   0.865420505042654
                               4.856066159002346   0.902576404040427];
    p_fp_p_dss{2} = 1.0e+02 * [1.886107656616930   3.600313957272261
                               4.342373715541079   4.089996366420240
                               1.998891511786369   1.102973173402852
                               4.767630817332042   1.400119347161953];
    p_fp_p_dss{3} = 1.0e+02 * [2.754512734382462   3.879126096733292
                               4.826685658804343   3.652821960654624
                               2.761711229540450   0.855223708793761
                               4.745691979350180   1.308546811147698];

    % Perform single calibration
    calib_test = alg.single_calib_fp_dr(calib_config.obj_A, ...
                                        calib_config.obj_R, ...
                                        class.calib.cb_w2p_c2e(calib_config), ...
                                        class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config), ...
                                        calib_config.obj_cb_geom, ...
                                        img_cbs, ...
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
