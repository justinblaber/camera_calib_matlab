classdef target_base < class.cb_geom.size_base & class.cb_geom.target_intf
    % This is the base class definition for a calibration board target
    % geometry.

    methods(Access = public)
        function obj = target_base(opts)
            obj@class.cb_geom.size_base(opts);
        end
    end
end
