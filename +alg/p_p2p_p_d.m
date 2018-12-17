function p_p_ds = p_p2p_p_d(p_ps, f_p_p2p_p_d, a, d)
    % Transforms points from pixel coordinates to distorted pixel
    % coordinates.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of pixel points
    %   f_p_p2p_p_d - function handle; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    %   d - array; Mx1 array of distortion coefficients
    %
    % Outputs:
    %   p_p_ds - array; Nx2 array of distorted pixel points

    % Apply transform to coordinates
    d_cell = num2cell(d);
    p_p_ds = f_p_p2p_p_d(p_ps(:, 1), ...
                         p_ps(:, 2), ...
                         a(1), ...
                         a(2), ...
                         a(3), ...
                         d_cell{:});
end
