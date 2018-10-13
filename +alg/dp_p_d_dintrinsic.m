function jacob = dp_p_d_dintrinsic(p_ws,f_p_w2p_p,f_dp_p_dh,R,t,f_dp_p_d_dargs,A,d)
    % Returns jacobian of distorted pixel points wrt intrinsics.
    %
    % Inputs:
    %   p_ws - array; Nx2 array of points in world coordinates    
    %   f_p_w2p_p - function handle; function which transforms world
    %           coordinates to pixel coordinates  
    %   f_dp_p_dh - function handle; derivative of p_w2p_p wrt homography
    %       parameters.
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector
    %   f_dp_p_d_dargs - function handle; derivative of p_p2p_p_d wrt its
    %       input arguments.
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    %
    % Outputs:
    %   jacob - array; 2*Nx(3+M) array. 
    %       Format of jacobian is:
    %
    %                dalpha dx_o dy_o dd_1, ..., dd_M
    %       dx_p_d_1
    %       dy_p_d_1
    %          .
    %          .
    %          .
    %       dx_p_d_N
    %       dy_p_d_N
   
    % Get homography
    H = A*[R(:,1) R(:,2) t];
        
    % Get p_p
    p_ps = f_p_w2p_p(p_ws,H);

    % Get dp_p_d_dh
    dp_p_dh = f_dp_p_dh(p_ws,H);
    dp_p_d_dp_p = alg.dp_p_d_dp_p(p_ps,f_dp_p_d_dargs{1},f_dp_p_d_dargs{2},A,d);
    dp_p_d_dh = dp_p_d_dp_p*dp_p_dh;
    
    % 1) alpha: [dp_p_d/dh]*[dh/dalpha] + [dp_p_d/dalpha]
    dh_dalpha = [R(1,1); R(2,1); 0; R(1,2); R(2,2); 0; t(1); t(2); 0];
    dp_p_d_dalpha = dp_p_d_dh*dh_dalpha + alg.dp_p_d_darg(p_ps,f_dp_p_d_dargs{3},A,d);
    
    % 2) x_o: [dp_p_d/dh]*[dh/dx_o] + [dp_p_d/dx_o]
    dh_dx_o = [R(3,1); 0; 0; R(3,2); 0; 0; t(3); 0; 0];
    dp_p_d_dx_o = dp_p_d_dh*dh_dx_o + alg.dp_p_d_darg(p_ps,f_dp_p_d_dargs{4},A,d);
    
    % 3) y_o: [dp_p_d/dh]*[dh/dy_o] + [dp_p_d/dy_o]
    dh_dy_o = [0; R(3,1); 0; 0; R(3,2); 0; 0; t(3); 0];
    dp_p_d_dy_o = dp_p_d_dh*dh_dy_o + alg.dp_p_d_darg(p_ps,f_dp_p_d_dargs{5},A,d);
    
    % 4) d: [dp_p_d/dd] - assumes p_p has no dependence on these parameters
    num_params_d = nargin(f_dp_p_d_dargs{1}) - 5;   
    dp_p_d_dd = zeros(2*size(p_ws,1),num_params_d);
    for i = 1:num_params_d
        dp_p_d_dd(:,i) = alg.dp_p_d_darg(p_ps,f_dp_p_d_dargs{i+5},A,d);
    end
    
    % Form jacobian
    jacob = [dp_p_d_dalpha dp_p_d_dx_o dp_p_d_dy_o dp_p_d_dd];
end
