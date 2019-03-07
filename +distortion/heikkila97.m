function sym_p_p2p_p_d = heikkila97()
    % This is the distortion model in heikkila97.
    %
    % Note that the function must have arguments which start with:
    %
    %   x_p, y_p, a_x, a_y, s, x_o, y_o
    %
    % in that order, and then are followed by distortion parameters.
    %
    % Inputs:
    %   None
    %
    % Outputs:
    %   sym_p_p2p_p_d - symbolic function; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.

    % Declare symbolic function
    syms sym_p_p2p_p_d(x_p, y_p, a_x, a_y, s, x_o, y_o, k1, k2, p1, p2)

    % Convert to normalized coordinates
    A = [a_x,   s, x_o;
           0, a_y, y_o;
           0,   0,   1];
    p_n = inv(A)*[x_p; y_p; 1]; %#ok<MINV>
    x_n = p_n(1, :);
    y_n = p_n(2, :);

    % Apply radial distortion
    x_n_r = x_n.*(1 + k1*(x_n.^2 + y_n.^2) + k2*(x_n.^2+y_n.^2).^2);
    y_n_r = y_n.*(1 + k1*(x_n.^2 + y_n.^2) + k2*(x_n.^2+y_n.^2).^2);

    % Distorted normalized points
    x_n_d = x_n_r + 2*p1*x_n.*y_n + p2*(3*x_n.^2 + y_n.^2);
    y_n_d = y_n_r + p1*(x_n.^2 + 3*y_n.^2) + 2*p2*x_n.*y_n;

    % Convert from distorted normalized points to distorted pixel points
    p_p_d = A*[x_n_d; y_n_d; 1];
    x_p_d = p_p_d(1, :);
    y_p_d = p_p_d(2, :);

    % Define symbolic function
    sym_p_p2p_p_d(x_p, y_p, a_x, a_y, s, x_o, y_o, k1, k2, p1, p2) = ...
        [x_p_d, y_p_d];
end
