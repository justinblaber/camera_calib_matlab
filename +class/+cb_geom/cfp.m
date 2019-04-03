classdef cfp < class.cb_geom.fp_base
    % This is the class definition for a centered four point box
    % calibration board geometry.

    properties(Access = private)
        height_fp               % float
        width_fp                % float
    end

    methods(Access = private)
        function h = get_fp_height(obj)
            h = obj.height_fp;
        end

        function w = get_fp_width(obj)
            w = obj.width_fp;
        end
    end

    methods(Access = public)
        function obj = cfp(opts)
            obj@class.cb_geom.fp_base(opts);

            obj.height_fp = opts.height_fp;
            obj.width_fp = opts.width_fp;
        end

        % Abstract methods -----------------------------------------------%

        function p_fp_ws = get_p_fp_ws(obj)
            % Get half height and half width of calibration board
            hh_cb = obj.get_cb_height()/2;
            hw_cb = obj.get_cb_width()/2;

            % Get half height and half width of four point box
            hh_fp = obj.get_fp_height()/2;
            hw_fp = obj.get_fp_width()/2;

            % Get four point box; assume they are centered on the
            % calibration board
            p_fp_ws = [hw_cb-hw_fp hh_cb-hh_fp;
                       hw_cb-hw_fp hh_cb+hh_fp;
                       hw_cb+hw_fp hh_cb-hh_fp;
                       hw_cb+hw_fp hh_cb+hh_fp];
        end
    end
end
