classdef (Abstract) fp_intf < handle
    % This is the interface for a calibration board four point geometry.

    methods(Abstract, Access = public)
        % Returns four points in world coordinates
        p_fp_ws = get_p_fp_ws(obj)
    end
end
