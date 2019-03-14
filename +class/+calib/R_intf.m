classdef (Abstract) R_intf < handle
    % This is the interface for a rotation matrix parameterization.

    methods(Abstract, Access = public)
        R = r2R(obj, r)
        r = R2r(obj, R)
        dR_dr = dR_dr(obj, r)
        num = get_num_params_r(obj)
        args = get_r_args(obj)
    end
end
