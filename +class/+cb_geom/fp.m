classdef (Abstract) fp < class.cb_geom.base
    % This is the class definition for a calibration board four point
    % geometry.

    methods(Access = public)
        function obj = fp(opts)
            obj@class.cb_geom.base(opts);
        end
    end

    methods(Abstract, Access = public)
        % Returns four points in world coordinates
        p_fp_ws = get_p_fp_ws(obj)
    end
end
