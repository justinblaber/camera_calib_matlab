classdef cdgrid_cfp < class.cb_geom.cdgrid & class.cb_geom.cfp
    % This is the class definition for a centered diamond grid calibration
    % board pattern with a centered four point box geometry.

    methods(Access = public)
        function obj = cdgrid_cfp(opts)
            obj@class.cb_geom.cdgrid(opts);
            obj@class.cb_geom.cfp(opts);
        end
    end
end
