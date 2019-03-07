function R = euler2R(euler)
    % Converts euler angles to rotation matrix. Assumes X-Y-Z rotation
    % sequence.
    %
    % Inputs:
    %   euler - array; 3x1 euler angles. Stored as:
    %       [theta_x; theta_y; theta_z]
    %
    % Outputs:
    %   R - array; 3x3 rotation matrix

    theta_x = euler(1);
    theta_y = euler(2);
    theta_z = euler(3);

    R_x = [1                0               0;
           0                cos(theta_x)   -sin(theta_x);
           0                sin(theta_x)    cos(theta_x)];
    R_y = [cos(theta_y)     0               sin(theta_y);
           0                1               0;
          -sin(theta_y)     0               cos(theta_y)];
    R_z = [cos(theta_z)    -sin(theta_z)    0;
           sin(theta_z)     cos(theta_z)    0;
           0                0               1];

    R = R_z*R_y*R_x;
end
