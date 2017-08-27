function [points_n, r_n] = p_n(R,t,points_w)
    % This will convert world points to normalized camera points 
    %
    % Inputs:
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation
    %   points_w - array; Nx2 array of points in world coordinates
    %
    % Outputs:
    %   points_s - array; Nx2 array of points in camera coordinates
    
    % Apply xform to points to get into camera coordinates    
    points_s = alg.p_s(R,t,points_w);    

    % Get normalized coordinates; also output r_n since this is commonly
    % used
    x_n = points_s(:,1)./points_s(:,3);
    y_n = points_s(:,2)./points_s(:,3);
    r_n = sqrt(x_n.^2 + y_n.^2);   
    
    % set output
    points_n = [x_n y_n];
end