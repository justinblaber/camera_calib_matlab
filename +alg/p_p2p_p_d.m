function p_p_ds = p_p2p_p_d(p_ps,f_p_p2p_p_d,A,d)
    % Transforms points from pixel coordinates to distorted pixel 
    % coordinates.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of pixel points
    %   f_p_p2p_p_d - function handle; describes the mapping between 
    %       pixel coordinates and distorted pixel coordinates.
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    % 
    % Outputs:
    %   p_p_ds - array; Nx2 array of distorted pixel points
    
    % Get camera matrix components
    alpha = A(1,1);
    x_o = A(1,3);
    y_o = A(2,3);
        
    % Apply transform to coordinates
    d_cell = num2cell(d);
    p_p_ds = f_p_p2p_p_d(p_ps(:,1), ...
                         p_ps(:,2), ...
                         alpha, ...
                         x_o, ...
                         y_o, ...
                         d_cell{:});
end