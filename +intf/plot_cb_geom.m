function plot_cb_geom(calib_config)
    % Wrapper function for debug.plot_cb_geom()
    %
    % Inputs:
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %
    % Outputs:
    %   None

    debug.plot_cb_geom(calib_config.obj_cb_geom);
end
