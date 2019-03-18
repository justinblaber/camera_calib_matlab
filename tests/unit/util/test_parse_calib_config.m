function test_parse_calib_config
    data.target              = 'checker';
    data.target_optimization = 'opencv';
    data.height_cb           = 400;
    data.width_cb            = 600;
    data.num_targets_height  = 10;
    data.num_targets_width   = 20;
    data.target_spacing      = 1;
    data.obj_cb_geom         = 'class.cb_geom.csgrid_cfp';
    data.calib_optimization  = 'distortion_refinement';

    [calib_config, data] = util.parse_calib_config(data); %#ok<ASGLU>

    % calib_config is subject to a lot of change... so only include a very
    % weak test here.

    % Assert
    assert(numel(fields(data)) == 0);
end
