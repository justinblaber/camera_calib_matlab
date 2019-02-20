classdef (Abstract) cb_pattern < class.cb
    % This is the class definition for a calibration board pattern.

    methods(Access = public)
        function obj = cb_pattern(opts)
            obj@class.cb(opts);
        end
    end

    methods(Abstract, Access = public)
        % Method to get calibration world points
        p_cb_ws = get_p_cb_ws(obj)

        % Method to get boundaries around calibration world points in world
        % coordinates
        boundary_ws = get_p_cb_w_boundaries(obj)
    end
end
