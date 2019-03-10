classdef fp_base < class.cb_geom.size_base & class.cb_geom.fp_intf
    % This is the base class definition for a calibration board four point
    % geometry.

    methods(Access = public)
        function obj = fp_base(opts)
            obj@class.cb_geom.size_base(opts);
        end
    end
end
