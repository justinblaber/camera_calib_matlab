classdef cb_cdgrid_cfp < class.cb_cdgrid & class.cb_cfp
    % This is the class definition for a centered diamond grid calibration
    % board pattern with a centered four point box.

    methods(Access = public)
        function obj = cb_cdgrid_cfp(opts)
            obj@class.cb_cdgrid(opts);
            obj@class.cb_cfp(opts);
        end
    end
end
