classdef (Abstract) target_intf < handle
    % This is the interface for a calibration board target geometry.

    methods(Abstract, Access = public)
        % Returns calibration world points.
        p_cb_ws = get_p_cb_ws(obj)

        % Returns boundaries around calibration world points in world
        % coordinates.
        boundary_ws = get_p_cb_w_boundaries(obj)
    end
end
