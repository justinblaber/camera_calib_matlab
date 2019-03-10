classdef cb_w2p_c2e < class.calib.cb_w2p_base
    % Mapping of calibration board points using "center of circle" to
    % "center of ellipse"

    methods(Access = public)
        function obj = cb_w2p_c2e(opts)
            obj@class.calib.cb_w2p_base(opts);
        end
    end

    methods(Access = public)
        % Abstract methods -----------------------------------------------%

        function p_cb_ps = p_cb_w2p_cb_p(obj, p_cb_ws, H)
            p_cb_ps = alg.apply_homography_c2e(p_cb_ws, H, obj.get_opts().circle_radius);
        end

        function jacob = dp_cb_p_dH(obj, p_cb_ws, H)
            jacob = alg.dp_dH_c2e(p_cb_ws, H, obj.get_opts().circle_radius);
        end

        function H_w2p = homography_cb_w2p(obj, p_cb_ws, p_cb_ps, cov)
            if ~exist('cov', 'var')
                H_w2p = alg.homography_c2e(p_cb_ws, p_cb_ps, obj.get_opts().circle_radius, obj.get_opts());
            else
                H_w2p = alg.homography_c2e(p_cb_ws, p_cb_ps, obj.get_opts().circle_radius, obj.get_opts(), cov);
            end
        end
    end
end
