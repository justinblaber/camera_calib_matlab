function sym_p_p2p_p_d = wang08()
    % This is the distortion model in wang08.
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
    syms sym_p_p2p_p_d(x_p, y_p, a, x_o, y_o, k1, k2, p, t)

    % Convert to normalized coordinates - this makes jacobian better
    % conditioned
    x_n = (x_p - x_o)/a;
    y_n = (y_p - y_o)/a;

    % Apply radial distortion
    x_n_r = x_n.*(1 + k1*(x_n.^2 + y_n.^2) + k2*(x_n.^2+y_n.^2).^2);
    y_n_r = y_n.*(1 + k1*(x_n.^2 + y_n.^2) + k2*(x_n.^2+y_n.^2).^2);

    % Distorted normalized points
    x_n_d = x_n_r./(-p*x_n_r + t*y_n_r + 1);
    y_n_d = y_n_r./(-p*x_n_r + t*y_n_r + 1);

    % Define symbolic function
    sym_p_p2p_p_d(x_p, y_p, a, x_o, y_o, k1, k2, p, t) = ...
        [a*x_n_d + x_o, ...
         a*y_n_d + y_o];
end
