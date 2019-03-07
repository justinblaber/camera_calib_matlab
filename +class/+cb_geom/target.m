classdef (Abstract) target < class.cb_geom.base
    % This is the class definition for a calibration board target geometry.

    methods(Access = public)
        function obj = target(opts)
            obj@class.cb_geom.base(opts);
        end
    end

    methods(Abstract, Access = public)
        % Returns calibration world points.
        p_cb_ws = get_p_cb_ws(obj)

        % Returns boundaries around calibration world points in world
        % coordinates.
        boundary_ws = get_p_cb_w_boundaries(obj)
    end
end
