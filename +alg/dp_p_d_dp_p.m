function jacob = dp_p_d_dp_p(p_ps,f_dp_p_d_dx_p,f_dp_p_d_dy_p,A,d)
    % This will compute the jacobian of distorted pixel points wrt pixel
    % points.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of pixel points
    %   f_dp_p_d_dx_p - function handle; derivative of p_p2p_p_d wrt x_p
    %   f_dp_p_d_dy_p - function handle; derivative of p_p2p_p_d wrt y_p
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    % 
    % Outputs:
    %   jacob - sparse array; 2*Nx2*N array.
    %       Format of jacobian is:
    %
    %                dx_p_1 dy_p_1 ... dx_p_N dy_p_N
    %       dx_p_d_1
    %       dy_p_d_1
    %          .
    %          .
    %          .
    %       dx_p_d_N
    %       dy_p_d_N
    
    % Get camera matrix components
    alpha = A(1,1);
    x_o = A(1,3);
    y_o = A(2,3);
    
    % Convert d to cell - used to pass distortion arguments into function
    % handle
    d_cell = num2cell(d);
    
    % Compute partial derivatives
    dp_p_d_dx_p = f_dp_p_d_dx_p(p_ps(:,1), ...
                                p_ps(:,2), ...
                                alpha, ...
                                x_o, ...
                                y_o, ...
                                d_cell{:});
    dp_p_d_dy_p = f_dp_p_d_dy_p(p_ps(:,1), ...
                                p_ps(:,2), ...
                                alpha, ...
                                x_o, ...
                                y_o, ...
                                d_cell{:});

    % TODO: find a better way to construct sparse diagonal jacobian,
    % as this is probably the bottleneck

    % Jacobian
    jacob_cell = mat2cell(sparse(reshape(vertcat(dp_p_d_dx_p',dp_p_d_dy_p'),2,[])),2,2*ones(1,size(p_ps,1)));
    jacob = blkdiag(jacob_cell{:});
end