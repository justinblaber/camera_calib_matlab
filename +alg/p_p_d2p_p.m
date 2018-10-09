function p_ps = p_p_d2p_p(p_p_ds,p_p_inits,A,f_p_n2p_n_d,f_dp_n_d_dx_n,f_dp_n_d_dy_n,d,opts)
    % Transforms points from distorted pixel coordinates to pixel
    % coordinates.
    %
    % Inputs:
    %   p_p_ds - array; Nx2 array of distorted pixel points
    %   p_p_inits - array; Nx2 array of initial guess of pixel points
    %   A - array; 3x3 camera matrix
    %   f_p_n2p_n_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   f_dp_n_d_dx_n - function handle; derivative of p_n_d wrt x_n.
    %   f_dp_n_d_dy_n - function handle; derivative of p_n_d wrt y_n.
    %   d - array; Mx1 array of distortion coefficients
    %   opts - struct;
    %       .p_p_d2p_p_it_cutoff - int; max number of iterations performed
    %           for transformation of distorted pixel coordinates to pixel
    %           coordinates.
    %       .p_p_d2p_p_norm_cutoff - scalar; cutoff for norm of difference
    %           of parameter vector for transformation of distorted pixel 
    %           coordinates to pixel coordinates.
    % 
    % Outputs:
    %   p_ps - array; Nx2 array of pixel points
        
    % Convert d to cell - used to pass distortion arguments into function
    % handle
    d_cell = num2cell(d);
    
    % Get initial x_p_bar and y_p_bar
    x_p_bar_inits = p_p_inits(:,1) - x_o;
    y_p_bar_inits = p_p_inits(:,2) - y_o;

    % Initialize parameter vector    
    num_points = size(p_p_inits,1);
    params = reshape([x_p_bar_inits y_p_bar_inits]',[],1);
    for it = 1:opts.p_p_d2p_p_it_cutoff
        % Get p_p_bar from params
        p_p_bars = reshape(params,2,[])';
        
        % Compute jacobian and residual
        dp_p_d_dx_p_bar = f_dp_n_d_dx_n(p_p_bars(:,1), ...
                                        p_p_bars(:,2), ...
                                        d_cell{:});
        dp_p_d_dy_p_bar = f_dp_n_d_dy_n(p_p_bars(:,1), ...
                                        p_p_bars(:,2), ...
                                        d_cell{:});
        
        % Jacobian - this is probably bottle neck
        jacob_cell = mat2cell(sparse(reshape(vertcat(dp_p_d_dx_p_bar',dp_p_d_dy_p_bar'),2,[])),2,2*ones(1,num_points));
        jacob = blkdiag(jacob_cell{:});
        
        % Residual 
        res = reshape(f_p_n2p_n_d(p_p_bars(:,1),p_p_bars(:,2),alpha,x_o,y_o,d_cell{:})',[],1) - reshape(p_p_ds',[],1);

        % Get and store update
        delta_params = -lscov(jacob,res);
        params = params + delta_params;

        % Exit if change in distance is small
        if norm(delta_params) < opts.p_p_d2p_p_norm_cutoff
            break
        end
    end

    % Get p_p
    p_p_bars = reshape(params,2,[])';
    p_ps = p_p_bars + [x_o, y_o];
end