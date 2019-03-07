classdef R_euler < class.calib.R
    % Euler parameterization of a rotation matrix.

    methods(Access = public)
        % Abstract methods -----------------------------------------------%

        function R = r2R(obj, r) %#ok<INUSL>
            R = alg.euler2R(r);
        end

        function r = R2r(obj, R) %#ok<INUSL>
            r = alg.R2euler(R);
        end

        function dR_dr = dR_dr(obj, r) %#ok<INUSL>
            dR_dr = alg.dR_deuler(r);
        end

        function num = get_num_params_r(obj) %#ok<MANU>
            num = 3;
        end

        function args = get_r_args(obj) %#ok<MANU>
            args = {'theta_x', 'theta_y', 'theta_z'};
        end
    end
end
