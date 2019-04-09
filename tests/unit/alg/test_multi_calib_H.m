function test_multi_calib_H
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration config
    calib_config = intf.load_calib_config(fullfile(tests_path, 'data', 'stereo', 'stereo.conf'));

    % Set images
    path_cbs(:, 1) = {fullfile(tests_path, 'data', 'stereo', 'left01.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'left02.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'left03.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'left04.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'left05.jpg')};
    path_cbs(:, 2) = {fullfile(tests_path, 'data', 'stereo', 'right01.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'right02.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'right03.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'right04.jpg'), ...
                      fullfile(tests_path, 'data', 'stereo', 'right05.jpg')};

    % Validate all calibration board images
    img_cbs(:, 1) = class.img.path.validate_similar_imgs(path_cbs(:, 1));
    img_cbs(:, 2) = class.img.path.validate_similar_imgs(path_cbs(:, 2));

    H_w2ps{1, 1} = 1.0e+02 * [ 0.002731469370804   0.000268807928425   2.273259300647205
                             -0.000196626129997   0.003421431524351   0.417444214459161
                             -0.000001326765819   0.000000679514802   0.010000000000000];
    H_w2ps{1, 2} = 1.0e+02 * [ 0.002269506611190   0.000174089672647   1.138447549721626
                             -0.000439866561588   0.003182773903243   0.627490403111680
                             -0.000002293649247   0.000000357605905   0.010000000000000];

    H_w2ps{2, 1} = 1.0e+02 * [-0.001161195782461   0.003400211429900   2.030916727520529
                             -0.003768341137265   0.000623242001038   3.581049733191729
                             -0.000004423764070  -0.000000366586920   0.010000000000000];
    H_w2ps{2, 2} = 1.0e+02 * [-0.001064373233360   0.002899160224940   0.832780344550567
                             -0.003619329346306   0.000305642417210   3.647065000446309
                             -0.000004243506204  -0.000001376417031   0.010000000000000];

    H_w2ps{3, 1} = 1.0e+02 * [ 0.003233462670783  -0.002111128451381   2.830884560967245
                              0.000978278454239   0.003151663236723   0.142219144800778
                             -0.000001272890108  -0.000001917910021   0.010000000000000];
    H_w2ps{3, 2} = 1.0e+02 * [ 0.002707741904945  -0.001872801656130   1.425077622188492
                              0.000581886800172   0.003053956822725   0.366022910944461
                             -0.000002529247622  -0.000001498281287   0.010000000000000];

    H_w2ps{4, 1} = 1.0e+02 * [ 0.003088730252816  -0.000386540989099   1.750996508001958
                             -0.000482037471330   0.003563970569286   0.754835644151225
                             -0.000001784655826  -0.000000990902765   0.010000000000000];
    H_w2ps{4, 2} =           [ 0.268604351507566  -0.028613923351618  46.507786326988906
                             -0.072784056743855   0.341501306924861  97.202996723484105
                             -0.000266293610181  -0.000078047343813   1.000000000000000];

    H_w2ps{5, 1} = 1.0e+02 * [-0.000421041041476   0.004013863534514   1.780952016938086
                              0.002671371534730  -0.000895977064294   0.931659268994508
                             -0.000003496975125   0.000000214881980   0.010000000000000];
    H_w2ps{5, 2} = 1.0e+02 * [-0.000348810566068   0.003325563655591   0.486975243362257
                              0.002453529294488  -0.001157086760920   1.149756637743471
                             -0.000003366186154  -0.000001067933393   0.010000000000000];

    % Get calibration object
    obj_calib = class.calib.base(calib_config.obj_A, ...
                                 calib_config.obj_R, ...
                                 class.calib.cb_w2p_p2p(calib_config), ... % point to point
                                 class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config), ...
                                 calib_config);

    % Perform multi calibration
    calib_test = alg.multi_calib_H(@alg.single_calib_H_dr, ...
                                   obj_calib, ...
                                   calib_config.obj_cb_geom, ...
                                   img_cbs, ...
                                   H_w2ps, ...
                                   calib_config);

    % Assert
    load(fullfile(tests_path, 'data', 'stereo', 'calib.mat'));
    for i = 1:numel(calib_test.cam)
        assert(all(all(abs(calib_test.cam(i).intrin.A - calib.cam(i).intrin.A) < 1e-4)));
        assert(all(all(abs(calib_test.cam(i).intrin.d - calib.cam(i).intrin.d) < 1e-4)));
        for j = 1:numel(calib_test.cam(i).extrin)
            assert(all(all(abs(calib_test.cam(i).extrin(j).R - calib.cam(i).extrin(j).R) < 1e-4)));
            assert(all(all(abs(calib_test.cam(i).extrin(j).t - calib.cam(i).extrin(j).t) < 1e-4)));
        end
    end

    clear

    % TODO: Add single image calibration (this requires principle point to
    % be fixed)
end
