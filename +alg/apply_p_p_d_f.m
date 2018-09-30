function p_p_d = apply_p_p_d_f(f_p_p_d,p_p,A,d)
    % Applies distortion function to input points
    %
    % Inputs:
    %   f_p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   p_p - array; Nx2 array of ideal points
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients corresponding to
    %       input symbolic function
    % 
    % Outputs:
    %   p_p_d - array; Nx2 array of distorted points

    % Validate function handles arguments
    util.validate_p_p_d_f_args(f_p_p_d);
     
    % Validate camera matrix
    if A(1,1) ~= A(2,2) || A(1,2) ~= 0 || A(2,1) ~= 0
        error('Camera matrix must have single focal point without skew.')
    end
    
    % Get camera matrix components
    alpha = A(1,1);
    x_o = A(1,3);
    y_o = A(2,3);   
    
    % Convert d to cell - used to pass distortion arguments into function
    % handle
    d_cell = num2cell(d);
        
    % Get x_p_bar and y_p_bar
    x_p_bar = p_p(:,1) - x_o;
    y_p_bar = p_p(:,2) - y_o;
    
    % Apply transform to coordinates
    p_p_d = f_p_p_d(x_p_bar,y_p_bar,alpha,x_o,y_o,d_cell{:});
end