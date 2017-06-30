function points_m = apply_full_model(A,distortion,R,t,points)
    % This will apply full camera model to input points.
    %
    % Inputs:
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]
    %   distortion - array; 4x1 distortions (radial and tangential)
    %       stored as: [beta1 beta2 beta3 beta4]
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation
    %   points - array; Nx2 array of points
    %
    % Outputs:
    %   points_m - array; Nx2 array of points
    
    % Apply xform to points to get into camera coordinates
    points_s = [R(:,1) R(:,2) t] * [points ones(size(points,1),1)]';
    x_s = points_s(1,:)';
    y_s = points_s(2,:)';
    z_s = points_s(3,:)';

    % Get normalized coordinates
    x_n = x_s./z_s;
    y_n = y_s./z_s;
    r_n = sqrt(x_n.^2 + y_n.^2);           
            
    % Get model points
    alpha_x = A(1,1);
    alpha_y = A(2,2);
    x_o = A(1,3);
    y_o = A(2,3);        
    beta1 = distortion(1);
    beta2 = distortion(2);
    beta3 = distortion(3);
    beta4 = distortion(4);        
    x_m = alpha_x*(x_n.*(1+beta1*r_n.^2+beta2*r_n.^4) + 2*beta3*x_n.*y_n + beta4*(r_n.^2+2*x_n.^2)) + x_o;
    y_m = alpha_y*(y_n.*(1+beta1*r_n.^2+beta2*r_n.^4) + beta3*(r_n.^2+2*y_n.^2) + 2*beta4*x_n.*y_n) + y_o; 
    
    % Set output
    points_m = [x_m y_m];
end