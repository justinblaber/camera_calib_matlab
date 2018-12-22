function test_load_stereo_calib_fp    
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Read calibration
    [calib, calib_config] = util.load_stereo_calib_fp(fullfile(tests_path, 'data', 'stereo1', 'calib.txt')); %#ok<ASGLU>

    % Left ---------------------------------------------------------------%
    
    % Assert calib
    assert(isfield(calib,'L'));
        
    % Assert intrin
    assert(isfield(calib.L,'intrin'));
    assert(isfield(calib.L.intrin,'A'));
    assert(isfield(calib.L.intrin,'d'));
    
    % Assert extrin
    assert(isfield(calib.L,'extrin'));
    assert(numel(calib.L.extrin) == 5);
    assert(isfield(calib.L.extrin,'img_path'));
    assert(isfield(calib.L.extrin,'R'));
    assert(isfield(calib.L.extrin,'t'));
    assert(isfield(calib.L.extrin,'p_fp_p_ds'));
    assert(isfield(calib.L.extrin,'p_cb_p_ds'));
    assert(isfield(calib.L.extrin,'cov_cb_p_ds'));
    assert(isfield(calib.L.extrin,'idx_valid'));
        
    % Right --------------------------------------------------------------%
    
    % Assert calib
    assert(isfield(calib,'R'));
    
    % Assert intrin
    assert(isfield(calib.R,'intrin'));
    assert(isfield(calib.R.intrin,'A'));
    assert(isfield(calib.R.intrin,'d'));
    
    % Assert extrin
    assert(isfield(calib.R,'extrin'));
    assert(numel(calib.R.extrin) == 5);
    assert(isfield(calib.R.extrin,'img_path'));
    assert(isfield(calib.R.extrin,'R'));
    assert(isfield(calib.R.extrin,'t'));
    assert(isfield(calib.R.extrin,'p_fp_p_ds'));
    assert(isfield(calib.R.extrin,'p_cb_p_ds'));
    assert(isfield(calib.R.extrin,'cov_cb_p_ds'));
    assert(isfield(calib.R.extrin,'idx_valid'));
        
    % R_s and t_s --------------------------------------------------------%
    
    assert(isfield(calib,'R_s'));
    assert(isfield(calib,'t_s'));
end