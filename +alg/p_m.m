function points_m = p_m(A,distortion,R,t,points_w)
    % This will apply the full camera model (same model used in Bouguet's 
    % camera calibration toolbox) to input points in world coordinates. The 
    % output coordinates are in image coordinates.
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
    %   points_w - array; Nx2 array of points in world coordinates
    %
    % Outputs:
    %   points_m - array; Nx2 array of points in image coordinates.
    
    % Get normalized coordinates
    [p_n, r_n] = alg.p_n(R,t,points_w);
    x_n = p_n(:,1);
    y_n = p_n(:,2);        
            
    % Get model points    
    x_m = A(1,1)*(x_n.*(1+distortion(1)*r_n.^2+distortion(2)*r_n.^4) + 2*distortion(3)*x_n.*y_n + distortion(4)*(r_n.^2+2*x_n.^2)) + A(1,3);
    y_m = A(2,2)*(y_n.*(1+distortion(1)*r_n.^2+distortion(2)*r_n.^4) + distortion(3)*(r_n.^2+2*y_n.^2) + 2*distortion(4)*x_n.*y_n) + A(2,3); 
    
    % Set output
    points_m = [x_m y_m];
end