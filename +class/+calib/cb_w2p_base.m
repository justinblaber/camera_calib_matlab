classdef cb_w2p_base < class.calib.cb_w2p_intf
    % This is the base class definition for the mapping between calibration
    % board world points and calibration board pixel points.

    properties(Access = private)
        opts
    end

    methods(Access = protected)
        function opts = get_opts(obj)
            opts = obj.opts;
        end
    end

    methods(Access = public)
        function obj = cb_w2p_base(opts)
            obj.opts = opts;
        end
    end
end
