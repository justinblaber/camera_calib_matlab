function test_parse_calib_config
    data.target_type = 'checker';
    data.num_targets_height = 10;
    data.num_targets_width = 20;
    data.target_spacing = 1;
        
    [calib_config, data] = util.parse_calib_config(data); %#ok<ASGLU>
    
    % calib_config is subject to a lot of change... so assert weakly or not
    % at all
    
    % Assert
    assert(numel(fields(data)) == 0);
end