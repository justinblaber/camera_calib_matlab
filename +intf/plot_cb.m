function plot_cb(calib_config)
    % Wrapper function for debug.plot_cb_class()
    %
    % Inputs:
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %
    % Outputs:
    %   None

    debug.plot_cb_class(calib_config.cb_class);
end
