function p_p_ds = p_w2p_p_d(p_ws, f_p_w2p_p, H, f_p_p2p_p_d, a, d)
    % Transforms points from world coordinates to distorted pixel
    % coordinates.
    %
    % Inputs:
    %   p_ws - array; Nx2 array of world points
    %   f_p_w2p_p - function handle; describes the mapping between world
    %       coordinates and pixel coordinates.
    %   H - array; 3x3 homography from world coordinates to pixel
    %       coordinates.
    %   f_p_p2p_p_d - function handle; describes the mapping between
    %       pixel coordinates and distorted pixel coordinates.
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    %   d - array; Mx1 array of distortion coefficients
    %
    % Outputs:
    %   p_p_ds - array; Nx2 array of distorted pixel points

    % Get pixel points
    p_ps = f_p_w2p_p(p_ws, H);

    % Get distorted pixel points
    p_p_ds = alg.p_p2p_p_d(p_ps, f_p_p2p_p_d, a, d);
end
