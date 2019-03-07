classdef base < handle
    % This is the base class definition for a calibration board geometry.

    properties(Access = private)
        height_cb               % float
        width_cb                % float
    end

    methods(Access = public)
        function obj = base(opts)
            obj.height_cb = opts.height_cb;
            obj.width_cb = opts.width_cb;
        end

        function h = get_cb_height(obj)
            h = obj.height_cb;
        end

        function w = get_cb_width(obj)
            w = obj.width_cb;
        end
    end
end
