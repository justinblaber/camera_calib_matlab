function test_load_single_calib_fp
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration
    calib = util.load_single_calib_fp(fullfile(tests_path, 'data', 'checker', 'calib.txt'));

    % Assert config
    assert(isfield(calib, 'config'));

    % Assert intrin
    assert(isfield(calib, 'intrin'));
    assert(isfield(calib.intrin, 'A'));
    assert(isfield(calib.intrin, 'd'));

    % Assert extrin
    assert(isfield(calib, 'extrin'));
    assert(numel(calib.extrin) == 3);
    assert(isfield(calib.extrin, 'img_cb'));
    assert(isfield(calib.extrin, 'R'));
    assert(isfield(calib.extrin, 't'));
    assert(isfield(calib.extrin, 'p_fp_p_ds'));
    assert(isfield(calib.extrin, 'p_cb_p_ds'));
    assert(isfield(calib.extrin, 'cov_cb_p_ds'));
    assert(isfield(calib.extrin, 'idx_valid'));
end
