classdef (Abstract) cb_fp < class.cb
    % This is the class definition for the four points on the calibration
    % board.

    methods(Access = public)
        function obj = cb_fp(opts)
            obj@class.cb(opts);
        end
    end

    methods(Abstract, Access = public)
        % Method to get four points in world coordinates
        p_fp_ws = get_p_fp_ws(obj)
    end
end
