function test_parse_single_calib
    data.A = [];
    data.d = [];

    data.img_path_1 = '';
    data.R_1 = [];
    data.t_1 = [];
    data.p_fp_p_ds_1 = [];
    data.p_cb_p_ds_1 = [];
    data.cov_cb_p_ds_1 = [];
    data.p_cb_p_d_ms_1 = [];
    data.idx_valid_1 = [];

    data.img_path_2 = '';
    data.R_2 = [];
    data.t_2 = [];
    data.p_fp_p_ds_2 = [];
    data.p_cb_p_ds_2 = [];
    data.cov_cb_p_ds_2 = [];
    data.p_cb_p_d_ms_2 = [];
    data.idx_valid_2 = [];

    [calib, data] = util.parse_single_calib(data);

    % Assert empty
    assert(numel(fields(data)) == 0);

    % Assert intrin
    assert(isfield(calib, 'intrin'));
    assert(isfield(calib.intrin, 'A'));
    assert(isfield(calib.intrin, 'd'));

    % Assert extrin
    assert(isfield(calib, 'extrin'));
    assert(numel(calib.extrin) == 2);
    assert(isfield(calib.extrin, 'img_cb'));
    assert(isfield(calib.extrin, 'R'));
    assert(isfield(calib.extrin, 't'));
    assert(isfield(calib.extrin, 'p_fp_p_ds'));
    assert(isfield(calib.extrin, 'p_cb_p_ds'));
    assert(isfield(calib.extrin, 'cov_cb_p_ds'));
    assert(isfield(calib.extrin, 'p_cb_p_d_ms'));
    assert(isfield(calib.extrin, 'idx_valid'));
end
