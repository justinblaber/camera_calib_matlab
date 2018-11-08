function sym_p_p2p_p_d = none()
    % This is the distortion free model.
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
    syms sym_p_p2p_p_d(x_p,y_p,a,x_o,y_o)
        
    % Define symbolic function
    sym_p_p2p_p_d(x_p,y_p,a,x_o,y_o) = [x_p, y_p];
end