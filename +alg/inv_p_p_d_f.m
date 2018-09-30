function p_p = inv_p_p_d_f(f_p_p_d,f_dp_p_d_dp_p_bar,p_p_init,p_p_d,A,d,opts)
    % Computes inverse of distortion function given function handles of 
    % distortion function and its derivatives.
    %
    % Inputs:
    %   f_p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   f_dp_p_d_dp_p_bar - function handle; derivative of f_p_p_d wrt
    %       p_p_bar
    %   p_p_init - array; Nx2 array of initial guess of ideal points
    %   p_p_d - array; Nx2 array of distorted points
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients corresponding to
    %       input symbolic function
    %   opts - struct;
    %       .inv_p_p_d_it_cutoff - int; number of iterations performed
    %           for inverse distortion refinement
    %       .inv_p_p_d_norm_cutoff - scalar; cutoff for norm of 
    %           difference of parameter vector for inverse distortion
    %           refinement
    % 
    % Outputs:
    %   p_p - array; Nx2 array of optimized ideal points

    % Validate function handles arguments
    util.validate_f_p_p_d_args(f_p_p_d);
    util.validate_f_p_p_d_args(f_dp_p_d_dp_p_bar);  
 
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
    
    % Get initial x_p_bar and y_p_bar
    x_p_bar_init = p_p_init(:,1) - x_o;
    y_p_bar_init = p_p_init(:,2) - y_o;

    % Initialize jacobian, residuals, and parameter vector
    num_points = size(p_p_init,1);
    jacob = zeros(num_points*2, num_points*2);
    res = zeros(num_points*2,1);
    p = reshape([x_p_bar_init y_p_bar_init]',[],1);
    for it = 1:opts.inv_p_p_d_it_cutoff
        % Compute jacobian and residual
        for i = 1:size(p_p_d,1)
            % Jacobian    
            jacob(2*i-1:2*i,2*i-1:2*i) = f_dp_p_d_dp_p_bar(p(2*i-1),p(2*i),alpha,x_o,y_o,d_cell{:}); 

            % Residual
            res(2*i-1:2*i) = f_p_p_d(p(2*i-1),p(2*i),alpha,x_o,y_o,d_cell{:}) - p_p_d(i,:);
        end

        % Get and store update
        delta_p = -lscov(jacob,res);
        p = p + delta_p;

        % Exit if change in distance is small
        if norm(delta_p) < opts.inv_p_p_d_norm_cutoff
            break
        end
    end

    % Get p_p
    p_p_bar = reshape(p,2,[])';
    p_p = p_p_bar + [x_o, y_o];
end