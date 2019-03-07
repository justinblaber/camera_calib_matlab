classdef A < handle
    % This is the base class definition for a camera matrix
    % parameterization.

    methods(Abstract, Access = public)
        A = a2A(obj, a)
        a = A2a(obj, A)
        dA_da = dA_da(obj, a)
        num = get_num_params_a(obj)
        args = get_a_args(obj)
    end
end
