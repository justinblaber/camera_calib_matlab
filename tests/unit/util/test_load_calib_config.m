function test_load_calib_config
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % calib_config is subject to a lot of change... so only include a very
    % weak test here.

    % Read calibration config
    calib_config = util.load_calib_config(fullfile(tests_path, 'data', 'circle', 'circle.conf')); %#ok<NASGU>
end
