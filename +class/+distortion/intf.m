classdef (Abstract) intf < handle
    % This is the interface for camera distortion.

    methods(Abstract, Access = public)
        num_params_d = get_num_params_d(obj)
        args = get_d_args(obj)
        jacob = dp_p_d_dp_p(obj, p_ps, A, d)
        jacob = dp_p_d_dA(obj, p_ps, A, d)
        jacob = dp_p_d_dd(obj, p_ps, A, d)
        p_p_ds = p_p2p_p_d(obj, p_ps, A, d)
        p_ps = p_p_d2p_p(obj, p_p_ds, p_ps_init, A, d)
    end
end
