function [obj_A, obj_R, obj_cb_w2p, obj_distortion] = calib_parameterization(calib_config)
    % Get appropriate calibration parameterization based on input
    % calib_config.
    %
    % Inputs:
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %
    % Outputs:
    %   obj_A - class.calib.A_intf;
    %   obj_R - class.calib.R_intf;
    %   obj_cb_w2p - class.calib.cb_w2p_intf;
    %   obj_distortion - class.distortion.intf;

    % Get A parameterization
    switch calib_config.A_parameterization
        case 'single_focal'
            obj_A = class.calib.A_sf();
        otherwise
            error(['Unknown A parameterization: "' calib_config.A_parameterization '"']);
    end

    % Get R parameterization
    switch calib_config.R_parameterization
        case 'euler'
            obj_R = class.calib.R_euler();
        otherwise
            error(['Unknown R parameterization: "' calib_config.R_parameterization '"']);
    end

    % Get world to pixel stuff
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            switch calib_config.target
                case 'checker'
                    obj_cb_w2p = class.calib.cb_w2p_p2p(calib_config);
                case 'circle'
                    obj_cb_w2p = class.calib.cb_w2p_c2e(calib_config);
                otherwise
                    error(['Cannot obtain world to pixel parameterization ' ...
                           'for ' calib_config.calib_optimization ' ' ...
                           'optimization with ' calib_config.target ' ' ...
                           'target.']);
            end
        otherwise
            error(['Cannot obtain world to pixel parameterization for ' ...
                   calib_config.calib_optimization ' optimization.']);
    end

    % Get distortion object
    obj_distortion = class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config);
end
