function xfm_p_bar2p_d_sym = heikkila97()
    % This is the distortion model in heikkila97. Note that the function 
    % must have arguments which start with:
    %
    %   x_p_bar, y_p_bar, a, x_o, y_o
    %
    % in that order, and then are followed by distortion parameters.
    %
    % Inputs:
    %   None
    %
    % Outputs:    
    %   xfm_p_bar2p_d_sym - symbolic function; describes mapping between 
    %       ideal pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    
    % Declare symbolic function
    syms xfm_p_bar2p_d_sym(x_p_bar,y_p_bar,a,x_o,y_o,k1,k2,p1,p2)
    
    % Define symbolic function
    xfm_p_bar2p_d_sym(x_p_bar,y_p_bar,a,x_o,y_o,k1,k2,p1,p2) = ...
        [x_p_bar.*(1 + k1.*(x_p_bar.^2 + y_p_bar.^2) + k2.*(x_p_bar.^2+y_p_bar.^2).^2) + 2*p1*x_p_bar.*y_p_bar + p2*(3*x_p_bar.^2 + y_p_bar.^2) + x_o, ...
         y_p_bar.*(1 + k1.*(x_p_bar.^2 + y_p_bar.^2) + k2.*(x_p_bar.^2+y_p_bar.^2).^2) + p1*(x_p_bar.^2 + 3*y_p_bar.^2) + 2*p2*x_p_bar.*y_p_bar + y_o];    
end