classdef base < class.calib.A_intf & class.calib.R_intf & class.calib.cb_w2p_intf & class.distortion.intf
    % This is the base class definition for a calibration.

    properties(Access = private)
        obj_A
        obj_R
        obj_cb_w2p
        obj_distortion
        opts
    end

    methods(Access = public)
        function obj = base(obj_A, obj_R, obj_cb_w2p, obj_distortion, opts)
            obj.obj_A = obj_A;
            obj.obj_R = obj_R;
            obj.obj_cb_w2p = obj_cb_w2p;
            obj.obj_distortion = obj_distortion;
            obj.opts = opts;
        end

        % Forward A interface --------------------------------------------%

        function obj_A = get_obj_A(obj)
            obj_A = obj.obj_A;
        end

        function A = a2A(obj, a)
            A = obj.obj_A.a2A(a);
        end

        function a = A2a(obj, A)
            a = obj.obj_A.A2a(A);
        end

        function dA_da = dA_da(obj, a)
            dA_da = obj.obj_A.dA_da(a);
        end

        function num = get_num_params_a(obj)
            num = obj.obj_A.get_num_params_a();
        end

        function args = get_a_args(obj)
            args = obj.obj_A.get_a_args();
        end

        % Forward R interface --------------------------------------------%

        function obj_R = get_obj_R(obj)
            obj_R = obj.obj_R;
        end

        function R = r2R(obj, r)
            R = obj.obj_R.r2R(r);
        end

        function r = R2r(obj, R)
            r = obj.obj_R.R2r(R);
        end

        function dR_dr = dR_dr(obj, r)
            dR_dr = obj.obj_R.dR_dr(r);
        end

        function num = get_num_params_r(obj)
            num = obj.obj_R.get_num_params_r();
        end

        function args = get_r_args(obj)
            args = obj.obj_R.get_r_args();
        end

        % Forward cb_w2p interface ---------------------------------------%

        function obj_cb_w2p = get_obj_cb_w2p(obj)
            obj_cb_w2p = obj.obj_cb_w2p;
        end

        function p_cb_ps = p_cb_w2p_cb_p(obj, p_cb_ws, H)
            p_cb_ps = obj.obj_cb_w2p.p_cb_w2p_cb_p(p_cb_ws, H);
        end

        function jacob = dp_cb_p_dH(obj, p_cb_ws, H)
            jacob = obj.obj_cb_w2p.dp_cb_p_dH(p_cb_ws, H);
        end

        function H_w2p = homography_cb_w2p(obj, p_cb_ws, p_cb_ps, cov)
            if ~exist('cov', 'var')
                H_w2p = obj.obj_cb_w2p.homography_cb_w2p(p_cb_ws, p_cb_ps);
            else
                H_w2p = obj.obj_cb_w2p.homography_cb_w2p(p_cb_ws, p_cb_ps, cov);
            end
        end

        % Forward distortion interface -----------------------------------%

        function obj_distortion = get_obj_distortion(obj)
            obj_distortion = obj.obj_distortion;
        end

        function num_params_d = get_num_params_d(obj)
            num_params_d = obj.obj_distortion.get_num_params_d();
        end

        function args = get_d_args(obj)
            args = obj.obj_distortion.get_d_args();
        end

        function jacob = dp_p_d_dp_p(obj, p_ps, A, d)
            jacob = obj.obj_distortion.dp_p_d_dp_p(p_ps, A, d);
        end

        function jacob = dp_p_d_dA(obj, p_ps, A, d)
            jacob = obj.obj_distortion.dp_p_d_dA(p_ps, A, d);
        end

        function jacob = dp_p_d_dd(obj, p_ps, A, d)
            jacob = obj.obj_distortion.dp_p_d_dd(p_ps, A, d);
        end

        function p_p_ds = p_p2p_p_d(obj, p_ps, A, d)
            p_p_ds = obj.obj_distortion.p_p2p_p_d(p_ps, A, d);
        end

        function p_ps = p_p_d2p_p(obj, p_p_ds, p_ps_init, A, d)
            p_ps = obj.obj_distortion.p_p_d2p_p(p_p_ds, p_ps_init, A, d);
        end

        % Homography stuff -----------------------------------------------%

        function H = ARt2H(obj, A, R, t) %#ok<INUSL>
            H = A*[R(:, 1) R(:, 2) t];
        end

        function jacob = dH_dA(obj, A, R, t) %#ok<INUSL>
            jacob = [R(1, 1)*eye(3), R(2, 1)*eye(3), R(3, 1)*eye(3); ...
                     R(1, 2)*eye(3), R(2, 2)*eye(3), R(3, 2)*eye(3); ...
                        t(1)*eye(3),    t(2)*eye(3),    t(3)*eye(3)];
        end

        function jacob = dH_dR(obj, A, R, t) %#ok<INUSD,INUSL>
            jacob = [       A, zeros(3), zeros(3);
                     zeros(3),        A, zeros(3);
                     zeros(3), zeros(3), zeros(3)];
        end

        function jacob = dH_dt(obj, A, R, t) %#ok<INUSD,INUSL>
            jacob = [zeros(3);
                     zeros(3);
                           A];
        end

        % Distorted calibration board pixel points -----------------------%

        function p_cb_p_d_ms = p_cb_w2p_cb_p_d(obj, p_cb_ws, R, t, A, d)
            % Get calibration board pixel points
            p_cb_ps = obj.p_cb_w2p_cb_p(p_cb_ws, obj.ARt2H(A, R, t));

            % Get model calibration board distorted pixel points
            p_cb_p_d_ms = obj.p_p2p_p_d(p_cb_ps, A, d);
        end

        function jacob = dp_cb_p_d_dintrinsic(obj, p_cb_ws, R, t, A, d)
            % Get homography
            H = obj.ARt2H(A, R, t);

            % Get calibration board pixel points
            p_cb_ps = obj.p_cb_w2p_cb_p(p_cb_ws, H);

            % Get dp_cb_p_d/dH
            dp_cb_p_dH = obj.dp_cb_p_dH(p_cb_ws, H);
            dp_cb_p_d_dp_cb_p = obj.dp_p_d_dp_p(p_cb_ps, A, d);
            dp_cb_p_d_dH = dp_cb_p_d_dp_cb_p*dp_cb_p_dH;

            % 1) a: [dp_cb_p_d/dH]*[dH/dA]*[dA/da] + [dp_cb_p_d/dA]*[dA/da]
            dH_dA = obj.dH_dA(A, R, t);
            dA_da = obj.dA_da(obj.A2a(A));
            dp_cb_p_d_dA = obj.dp_p_d_dA(p_cb_ps, A, d);
            dp_cb_p_d_da = dp_cb_p_d_dH*dH_dA*dA_da + dp_cb_p_d_dA*dA_da;

            % 2) d: [dp_cb_p_d/dd] - assumes p_cb_p has no dependence on these parameters
            dp_cb_p_d_dd = obj.dp_p_d_dd(p_cb_ps, A, d);

            % Form jacobian
            jacob = [dp_cb_p_d_da dp_cb_p_d_dd];
        end

        function jacob = dp_cb_p_d_dextrinsic(obj, p_cb_ws, R, t, A, d, dRt_dextrinsic)
            % Get homography
            H = obj.ARt2H(A, R, t);

            % Get calibration board pixel points
            p_cb_ps = obj.p_cb_w2p_cb_p(p_cb_ws, H);

            % Get dp_cb_p_d/dH
            dp_cb_p_dH = obj.dp_cb_p_dH(p_cb_ws, H);
            dp_cb_p_d_dp_cb_p = obj.dp_p_d_dp_p(p_cb_ps, A, d);
            dp_cb_p_d_dH = dp_cb_p_d_dp_cb_p*dp_cb_p_dH;

            % Get dH/dRt
            dH_dRt = [obj.dH_dR(A, R, t) obj.dH_dt(A, R, t)];

            % Form jacobian
            jacob = dp_cb_p_d_dH*dH_dRt*dRt_dextrinsic;
        end

        % Params stuff ---------------------------------------------------%

        function num_params_t = get_num_params_t(obj) %#ok<MANU>
            num_params_t = 3;
        end

        function num_params_i = get_num_params_i(obj)
            num_params_i = obj.get_num_params_a() + obj.get_num_params_d();
        end

        function num_params_e = get_num_params_e(obj)
            num_params_e = obj.get_num_params_r() + obj.get_num_params_t();
        end

        function [param, params] = pop_param(obj, params, num_param) %#ok<INUSL>
            param = params(1:num_param);
            params(1:num_param) = [];
        end

        function print_param(obj, s_param, idx, params, cov_params, suffix)
            util.verbose_fprintf([pad(['    ' s_param ': '], 13) sprintf('% 10.4f', params(idx)) ' +- ' sprintf('% 8.4f', 3*sqrt(cov_params(idx, idx))) suffix], 1, obj.get_opts());
        end
    end

    methods(Access = protected)
        function opts = get_opts(obj)
            opts = obj.opts;
        end
    end
end
