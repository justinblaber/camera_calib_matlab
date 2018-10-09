function sym_p_n2p_n_d = wang08()
    % This is the distortion model in wang08. Note that the function 
    % must have arguments which start with:
    %
    %   x_n, y_n
    %
    % in that order, and then are followed by distortion parameters.
    %
    % Inputs:
    %   None
    %
    % Outputs:    
    %   sym_p_n2p_n_d - symbolic function; describes mapping between 
    %       normalized coordinates and distorted normalized coordinates.
    
    % Declare symbolic function
    syms sym_p_n2p_n_d(x_n,y_n,k1,k2,p,t)
    
    % Radial distortion
    x_n_r = x_n.*(1 + k1.*(x_n.^2 + y_n.^2) + k2.*(x_n.^2+y_n.^2).^2);
    y_n_r = y_n.*(1 + k1.*(x_n.^2 + y_n.^2) + k2.*(x_n.^2+y_n.^2).^2);
    
    % Distorted normalized points
    x_n_d = x_n_r./(-p*x_n_r + t*y_n_r + 1);
    y_n_d = y_n_r./(-p*x_n_r + t*y_n_r + 1);
    
    % Define symbolic function
    sym_p_n2p_n_d(x_n,y_n,k1,k2,p,t) = [x_n_d, y_n_d];    
end