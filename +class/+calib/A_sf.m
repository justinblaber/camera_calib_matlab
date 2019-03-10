classdef A_sf < class.calib.A_intf
    % Single focal length parameterization of a camera matrix.

    methods(Access = public)
        % Abstract methods -----------------------------------------------%

        function A = a2A(obj, a) %#ok<INUSL>
            A = [a(1),    0, a(2);
                    0, a(1), a(3);
                    0,    0,   1];
        end

        function a = A2a(obj, A) %#ok<INUSL>
            if any(A([2 3 4 6]) ~= 0) || A(9) ~= 1 || ~isequaln(A(1), A(5))
                error('Invalid camera matrix for conversion to camera vector');
            end

            a = A([1 7 8])';
        end

        function dA_da = dA_da(obj, a) %#ok<INUSD>
            dA_da = [1 0 0;
                     0 0 0;
                     0 0 0;
                     0 0 0;
                     1 0 0;
                     0 0 0;
                     0 1 0;
                     0 0 1;
                     0 0 0];
        end

        function num = get_num_params_a(obj) %#ok<MANU>
            num = 3;
        end

        function args = get_a_args(obj) %#ok<MANU>
            args = {'alpha', 'x_o', 'y_o'};
        end
    end
end
