function jacob = dR_deuler(euler)
    % Returns jacobian of rotation wrt euler angles. Assumes Z-Y-X rotation 
    % sequence.
    %
    % Inputs:
    %   euler - array; 3x1 euler angles. Stored as: 
    %       [theta_x; theta_y; theta_z]
    %
    % Outputs:
    %   jacob - array; 9x3 array of jacobian evaluated at input euler
    %       angles. Format of jacobian is:
    %
    %               dtheta_x dtheta_y dtheta_z
    %       dr11    
    %       dr12
    %       dr13
    %       dr21
    %       dr22
    %       dr23
    %       dr31
    %       dr32
    %       dr33
       
    % TODO: Add checks for degenerate cases, although these will basically
    % never happen with real data.
    
    jacob = [0                                                                        -sin(euler(2))*cos(euler(3))                -cos(euler(2))*sin(euler(3));
             0                                                                        -sin(euler(2))*sin(euler(3))                 cos(euler(2))*cos(euler(3));
             0                                                                        -cos(euler(2))                               0;
             sin(euler(1))*sin(euler(3))+cos(euler(1))*sin(euler(2))*cos(euler(3))     sin(euler(1))*cos(euler(2))*cos(euler(3))  -cos(euler(1))*cos(euler(3))-sin(euler(1))*sin(euler(2))*sin(euler(3));
            -sin(euler(1))*cos(euler(3))+cos(euler(1))*sin(euler(2))*sin(euler(3))     sin(euler(1))*cos(euler(2))*sin(euler(3))  -cos(euler(1))*sin(euler(3))+sin(euler(1))*sin(euler(2))*cos(euler(3));
             cos(euler(1))*cos(euler(2))                                              -sin(euler(1))*sin(euler(2))                 0;
             cos(euler(1))*sin(euler(3))-sin(euler(1))*sin(euler(2))*cos(euler(3))     cos(euler(1))*cos(euler(2))*cos(euler(3))   sin(euler(1))*cos(euler(3))-cos(euler(1))*sin(euler(2))*sin(euler(3));
            -cos(euler(1))*cos(euler(3))-sin(euler(1))*sin(euler(2))*sin(euler(3))     cos(euler(1))*cos(euler(2))*sin(euler(3))   sin(euler(1))*sin(euler(3))+cos(euler(1))*sin(euler(2))*cos(euler(3));
            -sin(euler(1))*cos(euler(2))                                              -cos(euler(1))*sin(euler(2))                 0];
end