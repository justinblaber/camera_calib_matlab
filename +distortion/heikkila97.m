function sym_p_p2p_p_d = heikkila97()
    % This is the distortion model in heikkila97. 
    %
    % Note that the function must have arguments which start with:
    %
    %   x_p, y_p, a, x_o, y_o
    %
    % in that order, and then are followed by distortion parameters.
    %
    % "a" is alpha and it's assumed there's a single focal length (i.e.
    % square pixels).
    %
    % Inputs:
    %   None
    %
    % Outputs:    
    %   sym_p_p2p_p_d - symbolic function; describes the mapping between 
    %       pixel coordinates and distorted pixel coordinates.
    
    % Declare symbolic function
    syms sym_p_p2p_p_d(x_p,y_p,a,x_o,y_o,k1,k2,p1,p2)
    
    % Remove principle point
    x_p_bar = x_p - x_o;
    y_p_bar = y_p - y_o;    
    
    % Apply radial distortion
    x_p_r_bar = x_p_bar.*(1 + k1*(x_p_bar.^2 + y_p_bar.^2) + k2*(x_p_bar.^2+y_p_bar.^2).^2);
    y_p_r_bar = y_p_bar.*(1 + k1*(x_p_bar.^2 + y_p_bar.^2) + k2*(x_p_bar.^2+y_p_bar.^2).^2);
    
    % Define symbolic function
    sym_p_p2p_p_d(x_p,y_p,a,x_o,y_o,k1,k2,p1,p2) = ...
        [x_p_r_bar + 2*p1*x_p_bar.*y_p_bar + p2*(3*x_p_bar.^2 + y_p_bar.^2) + x_o, ...
         y_p_r_bar + p1*(x_p_bar.^2 + 3*y_p_bar.^2) + 2*p2*x_p_bar.*y_p_bar + y_o];
end