classdef (Abstract) size_intf < handle
    % This is the interface for a calibration board size geometry

    methods(Abstract, Access = public)
        h = get_cb_height(obj)
        w = get_cb_width(obj)
    end
end
