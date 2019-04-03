function test_parse_single_cam
    data.A = [];
    data.d = [];

    data.obj_img1 = '';
    data.state_img1 = '';
    data.R1 = [];
    data.t1 = [];
    data.p_cb_p_ds1 = [];
    data.cov_cb_p_ds1 = [];
    data.p_cb_p_d_ms1 = [];
    data.idx_valid1 = [];
    data.p_fp_p_ds1 = [];

    data.obj_img2 = '';
    data.state_img2 = '';
    data.R2 = [];
    data.t2 = [];
    data.p_cb_p_ds2 = [];
    data.cov_cb_p_ds2 = [];
    data.p_cb_p_d_ms2 = [];
    data.idx_valid2 = [];
    data.p_fp_p_ds2 = [];

    data.R_1 = [];
    data.t_1 = [];

    [cam, data] = util.parse_single_cam(data);

    % Assert empty
    assert(numel(fields(data)) == 0);

    % Assert intrin
    assert(isfield(cam, 'intrin'));
    assert(isfield(cam.intrin, 'A'));
    assert(isfield(cam.intrin, 'd'));

    % Assert extrin
    assert(isfield(cam, 'extrin'));
    assert(numel(cam.extrin) == 2);
    assert(isfield(cam.extrin, 'img_cb'));
    assert(isfield(cam.extrin, 'R'));
    assert(isfield(cam.extrin, 't'));
    assert(isfield(cam.extrin, 'p_cb_p_ds'));
    assert(isfield(cam.extrin, 'cov_cb_p_ds'));
    assert(isfield(cam.extrin, 'p_cb_p_d_ms'));
    assert(isfield(cam.extrin, 'idx_valid'));
    assert(isfield(cam.extrin, 'p_fp_p_ds'));

    % Assert relative extrinsics
    assert(isfield(cam, 'R_1'));
    assert(isfield(cam, 't_1'));
end
