function p_p_ds = p_p2p_p_d(p_ps,f_p_p_bar2p_p_d,A,d)
    % Transforms points from ideal pixels to distorted pixels.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of ideal points
    %   f_p_p_bar2p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    % 
    % Outputs:
    %   p_p_ds - array; Nx2 array of distorted points
    
    % Get camera matrix components
    alpha = A(1,1);
    x_o = A(1,3);
    y_o = A(2,3);   
    
    % Convert d to cell - used to pass distortion arguments into function
    % handle
    d_cell = num2cell(d);
        
    % Get x_p_bar and y_p_bar
    x_p_bars = p_ps(:,1) - x_o;
    y_p_bars = p_ps(:,2) - y_o;
    
    % Apply transform to coordinates
    p_p_ds = f_p_p_bar2p_p_d(x_p_bars, ...
                             y_p_bars, ...
                             alpha, ...
                             x_o, ...
                             y_o, ...
                             d_cell{:});
end