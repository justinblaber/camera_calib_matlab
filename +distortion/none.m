function sym_p_p2p_p_d = none()
    % This is the distortion free model.
    %
    % Note that the function must have arguments which start with:
    %
    %   x_p, y_p, a11, a21, a31, a12, a22, a32, a13, a23, a33, d1, ..., dN
    %
    % in that order.
    %
    % Inputs:
    %   None
    %
    % Outputs:
    %   sym_p_p2p_p_d - symbolic function; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.

    % Declare symbolic function
    syms sym_p_p2p_p_d(x_p, y_p, a11, a21, a31, a12, a22, a32, a13, a23, a33)

    % Define symbolic function
    sym_p_p2p_p_d(x_p, y_p, a11, a21, a31, a12, a22, a32, a13, a23, a33) = [x_p, y_p];
end
