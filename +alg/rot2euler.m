function euler = rot2euler(R)
    % Converts rotation matrix to euler angles. Assumes X-Y-Z rotation 
    % sequence.
    %
    % Inputs:
    %   R - array; 3x3 rotation matrix
    %))];
    % Outputs:
    %   euler - array; 3x1 euler angles. Stored as: 
    %       [theta_x; theta_y; theta_z]
    
    euler = [atan2(R(3,2),R(3,3));
             atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
             atan2(R(2,1),R(1,1))];
end