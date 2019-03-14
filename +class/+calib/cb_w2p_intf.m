classdef (Abstract) cb_w2p_intf < handle
    % This is the interface for the mapping between calibration board world
    % points and calibration board pixel points.

    methods(Abstract, Access = public)
        p_cb_ps = p_cb_w2p_cb_p(obj, p_cb_ws, H)
        jacob = dp_cb_p_dH(obj, p_cb_ws, H)
        H_w2p = homography_cb_w2p(obj, p_cb_ws, p_cb_ps, cov)
    end
end
