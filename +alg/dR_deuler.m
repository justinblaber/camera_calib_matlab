function jacob = dR_deuler(euler)
    % Returns jacobian of rotation wrt euler angles. Assumes X-Y-Z rotation
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
    %       dR11
    %       dR21
    %       dR31
    %       dR12
    %       dR22
    %       dR32
    %       dR13
    %       dR23
    %       dR33

    % TODO: Add checks for degenerate cases (gimbal lock?), although this
    % shouldn't really happen with real data... I think...

    c_x = cos(euler(1));
    c_y = cos(euler(2));
    c_z = cos(euler(3));

    s_x = sin(euler(1));
    s_y = sin(euler(2));
    s_z = sin(euler(3));

    jacob = [0                      -s_y*c_z        -c_y*s_z;
             0                      -s_y*s_z         c_y*c_z;
             0                      -c_y             0;
             s_x*s_z+c_x*s_y*c_z     s_x*c_y*c_z    -c_x*c_z-s_x*s_y*s_z;
            -s_x*c_z+c_x*s_y*s_z     s_x*c_y*s_z    -c_x*s_z+s_x*s_y*c_z;
             c_x*c_y                -s_x*s_y         0;
             c_x*s_z-s_x*s_y*c_z     c_x*c_y*c_z     s_x*c_z-c_x*s_y*s_z;
            -c_x*c_z-s_x*s_y*s_z     c_x*c_y*s_z     s_x*s_z+c_x*s_y*c_z;
            -s_x*c_y                -c_x*s_y         0];
end
