classdef cb_w2p < handle
    % This is the base class definition for the mapping between calibration
    % board world points and calibration board pixel points.

    properties(Access = private)
        opts
    end

    methods(Access = public)
        function obj = cb_w2p(opts)
            obj.opts = opts;
        end
    end

    methods(Access = protected)
        function opts = get_opts(obj)
            opts = obj.opts;
        end
    end

    methods(Abstract, Access = public)
        p_cb_ps = p_cb_w2p_cb_p(obj, p_cb_ws, H)
        jacob = dp_cb_p_dH(obj, p_cb_ws, H)
        H_w2p = homography_cb_w2p(obj, p_cb_ws, p_cb_ps, cov)
    end
end
