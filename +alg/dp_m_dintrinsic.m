function jacob = dp_m_dintrinsic(A,distortion,R,t,points_w)
    % Returns jacobian of model points wrt camera parameters evaluated at
    % points in world coordinates.
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
    %   points_w - array; Nx2 array of points in world coordinates
    %
    % Outputs:
    %   jacob - array; 2*Nx8 array. 
    %       Format of jacobian is:
    %
    %               dalpha_x dalpha_y dx_o dy_o dbeta_1 dbeta_2 dbeta_3 dbeta_4
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
    
    % Get normalized coordinates
    [p_n, r_n] = alg.p_n(R,t,points_w);
    x_n = p_n(:,1);
    y_n = p_n(:,2);          
    
    % Get jacobian
    jacob = vertcat([x_n.*(1+distortion(1)*r_n.^2+distortion(2)*r_n.^4) + 2*distortion(3)*x_n.*y_n + distortion(4)*(r_n.^2 + 2*x_n.^2), ...         
                     zeros(num_points,1), ...                                                                     
                     ones(num_points,1), ...
                     zeros(num_points,1), ...
                     A(1,1)*x_n.*r_n.^2, ...
                     A(1,1)*x_n.*r_n.^4, ...
                     2*A(1,1).*x_n.*y_n, ...
                     A(1,1)*(r_n.^2+2*x_n.^2)], ...
                    [zeros(num_points,1), ...
                     y_n.*(1+distortion(1)*r_n.^2+distortion(2)*r_n.^4) + distortion(3)*(r_n.^2 + 2*y_n.^2) + 2*distortion(4)*x_n.*y_n, ...
                     zeros(num_points,1), ...
                     ones(num_points,1), ...
                     A(2,2)*y_n.*r_n.^2, ...
                     A(2,2)*y_n.*r_n.^4, ...
                     A(2,2)*(r_n.^2+2*y_n.^2), ...
                     2*A(2,2)*x_n.*y_n]);
end