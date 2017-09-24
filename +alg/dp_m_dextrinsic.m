function jacob = dp_m_dextrinsic(A,distortion,R,t,dRt_dm,points_w)
    % Returns jacobian of model points wrt extrinsic parameters evaluated
    % at points in world coordinates.
    %
    % Inputs:
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]
    %   distortion - array; 4x1 distortions (radial and tangential)
    %       stored as: [beta1; beta2; beta3; beta4]
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation
    %   dRt_dm - array; jacobian of rotation and translation with respect
    %       to euler angles and translation evaluated at points_w.
    %   points_w - array; Nx2 array of points in world coordinates
    %
    % Outputs:
    %   jacob - array; 2*num_pointsx6 array. 
    %       Format of jacobian is:
    %
    %               dtheta_x dtheta_y dtheta_z dtx dty dtz
    %       dx_m_1
    %          .
    %          .
    %          .
    %       dx_m_N
    %       dy_m_1
    %          .
    %          .
    %          .
    %       dy_m_N
           
    % Get number of points
    num_points = size(points_w,1);    
    
    % Get points in camera coordinates
    p_s = alg.p_s(R,t,points_w);
    x_s = p_s(:,1);
    y_s = p_s(:,2);
    z_s = p_s(:,3);

    % Get normalized coordinates
    [p_n, r_n] = alg.p_n(R,t,points_w);
    x_n = p_n(:,1);
    y_n = p_n(:,2);    
    
    % Get jacobian of normalized coords wrt R and t
    dx_n_dRt = [points_w(:,1)./z_s zeros(num_points,1) -(points_w(:,1).*x_s)./z_s.^2 points_w(:,2)./z_s zeros(num_points,1) -(points_w(:,2).*x_s)./z_s.^2 1./z_s zeros(num_points,1) -x_s./z_s.^2];
    dy_n_dRt = [zeros(num_points,1) points_w(:,1)./z_s -(points_w(:,1).*y_s)./z_s.^2 zeros(num_points,1) points_w(:,2)./z_s -(points_w(:,2).*y_s)./z_s.^2 zeros(num_points,1) 1./z_s -y_s./z_s.^2];

    % Get jacobian of normalized coords wrt euler angles and t
    dx_n_dm = dx_n_dRt*dRt_dm;
    dy_n_dm = dy_n_dRt*dRt_dm;
    dr_n_dm = (x_n.*dx_n_dm + y_n.*dy_n_dm)./r_n;
    
    % Get jacobian of model points wrt euler angles and t
    jacob = vertcat(A(1,1)*(dx_n_dm.*(1+distortion(1)*r_n.^2+distortion(2)*r_n.^4)+x_n.*(2*distortion(1)*r_n.*dr_n_dm+4*distortion(2)*r_n.^3.*dr_n_dm)+2*distortion(3)*(dx_n_dm.*y_n+x_n.*dy_n_dm)+distortion(4)*(2*r_n.*dr_n_dm+4*x_n.*dx_n_dm)), ...
                    A(2,2)*(dy_n_dm.*(1+distortion(1)*r_n.^2+distortion(2)*r_n.^4)+y_n.*(2*distortion(1)*r_n.*dr_n_dm+4*distortion(2)*r_n.^3.*dr_n_dm)+distortion(3)*(2*r_n.*dr_n_dm+4*y_n.*dy_n_dm)+2*distortion(4)*(dx_n_dm.*y_n+x_n.*dy_n_dm)));
end