classdef cb_w2p_p2p < class.calib.cb_w2p
    % Mapping of calibration board points using "center of point" to
    % "center of point"

    methods(Access = public)
        function obj = cb_w2p_p2p(opts)
            obj@class.calib.cb_w2p(opts);
        end
    end

    methods(Access = public)
        % Abstract methods -----------------------------------------------%

        function p_cb_ps = p_cb_w2p_cb_p(obj, p_cb_ws, H) %#ok<INUSL>
            p_cb_ps = alg.apply_homography_p2p(p_cb_ws, H);
        end

        function jacob = dp_cb_p_dH(obj, p_cb_ws, H) %#ok<INUSL>
            jacob = alg.dp_dH_p2p(p_cb_ws, H);
        end

        function H_w2p = homography_cb_w2p(obj, p_cb_ws, p_cb_ps, cov)
            if ~exist('cov', 'var')
                H_w2p = alg.homography_p2p(p_cb_ws, p_cb_ps, obj.get_opts());
            else
                H_w2p = alg.homography_p2p(p_cb_ws, p_cb_ps, obj.get_opts(), cov);
            end
        end
    end
end
