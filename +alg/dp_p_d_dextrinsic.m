function jacob = dp_p_d_dextrinsic(p_ws,f_p_w2p_p,f_dp_p_dh,R,t,f_dp_p_d_dargs,A,d,drt_dm)
    % Returns jacobian of distorted pixel points wrt extrinsics.
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
    %   drt_dm - array; 12x6 array containing jacobian of "rt" wrt
    %       extrinsics.
    %
    % Outputs:
    %   jacob - array; 2*Nx(3+M) array. 
    %       Format of jacobian is:
    %
    %                dtheta_x dtheta_y dtheta_z t_x t_y t_z
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
    
    % Get dh_drt
    dh_drt = alg.dh_drt(A);
    
    % Form jacobian
    jacob = dp_p_d_dh*dh_drt*drt_dm;
end
