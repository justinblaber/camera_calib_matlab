classdef cb_csgrid_cfp < class.cb_csgrid & class.cb_cfp
    % This is the class definition for a centered square grid calibration
    % board pattern with a centered four point box.

    methods(Access = public)
        function obj = cb_csgrid_cfp(opts)
            obj@class.cb_csgrid(opts);
            obj@class.cb_cfp(opts);
        end
    end
end
