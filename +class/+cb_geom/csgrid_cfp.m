classdef csgrid_cfp < class.cb_geom.csgrid & class.cb_geom.cfp
    % This is the class definition for a centered square grid calibration
    % board pattern with a centered four point box geometry.

    methods(Access = public)
        function obj = csgrid_cfp(opts)
            obj@class.cb_geom.csgrid(opts);
            obj@class.cb_geom.cfp(opts);
        end
    end
end
