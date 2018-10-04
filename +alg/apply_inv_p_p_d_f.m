function p_p = apply_inv_p_p_d_f(f_p_p_d,f_dp_p_d_dx_p_bar,f_dp_p_d_dy_p_bar,p_p_init,p_p_d,A,d,opts)
    % Computes inverse of distortion function given function handles of 
    % distortion function and its derivatives.
    %
    % Inputs:
    %   f_p_p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   f_dp_p_d_dx_p_bar - function handle; derivative of f_p_p_d wrt
    %       x_p_bar
    %   f_dp_p_d_dy_p_bar - function handle; derivative of f_p_p_d wrt
    %       y_p_bar
    %   p_p_init - array; Nx2 array of initial guess of ideal points
    %   p_p_d - array; Nx2 array of distorted points
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients corresponding to
    %       input symbolic function
    %   opts - struct;
    %       .apply_inv_p_p_d_it_cutoff - int; max number of iterations 
    %           performed for inverse distortion refinement
    %       .apply_inv_p_p_d_norm_cutoff - scalar; cutoff for norm of 
    %           difference of parameter vector for inverse distortion
    %           refinement
    % 
    % Outputs:
    %   p_p - array; Nx2 array of optimized ideal points

    % Validate function handles arguments
    util.validate_p_p_d_f_args(f_p_p_d);
    util.validate_p_p_d_f_args(f_dp_p_d_dx_p_bar);  
    util.validate_p_p_d_f_args(f_dp_p_d_dy_p_bar);  
 
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

    % Initialize parameter vector    
    num_points = size(p_p_init,1);
    p = reshape([x_p_bar_init y_p_bar_init]',[],1);
    for it = 1:opts.apply_inv_p_p_d_it_cutoff
        % Get p_p_bar from p
        p_p_bar = reshape(p,2,[])';
        
        % Compute jacobian and residual
        dx_p_d_dp_p_bar = f_dp_p_d_dx_p_bar(p_p_bar(:,1),p_p_bar(:,2),alpha,x_o,y_o,d_cell{:});
        dy_p_d_dp_p_bar = f_dp_p_d_dy_p_bar(p_p_bar(:,1),p_p_bar(:,2),alpha,x_o,y_o,d_cell{:});
        
        % Jacobian - this is probably bottle neck
        jacob_cell = mat2cell(sparse(reshape(vertcat(dx_p_d_dp_p_bar',dy_p_d_dp_p_bar'),2,[])),2,2*ones(1,num_points));
        jacob = blkdiag(jacob_cell{:});
        
        % Residual 
        res = reshape(f_p_p_d(p_p_bar(:,1),p_p_bar(:,2),alpha,x_o,y_o,d_cell{:})',[],1) - reshape(p_p_d',[],1);

        % Get and store update
        delta_p = -lscov(jacob,res);
        p = p + delta_p;

        % Exit if change in distance is small
        if norm(delta_p) < opts.apply_inv_p_p_d_norm_cutoff
            break
        end
    end

    % Get p_p
    p_p_bar = reshape(p,2,[])';
    p_p = p_p_bar + [x_o, y_o];
end