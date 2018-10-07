function p_ps = xfm_p_d2p(f_xfm_p_bar2p_d,f_df_xfm_p_bar2p_d_dx_p_bar,f_df_xfm_p_bar2p_d_dy_p_bar,p_p_inits,p_p_ds,A,d,opts)
    % Transforms points from distorted pixels to ideal pixels.
    %
    % Inputs:
    %   f_xfm_p_bar2p_d - function handle; describes mapping between ideal 
    %       pixel coordinates (with principle point subtracted) and 
    %       distorted pixel coordinates.
    %   f_df_xfm_p_bar2p_d_dx_p_bar - function handle; derivative of 
    %       f_xfm_p_bar2p_d wrt x_p_bar
    %   f_df_xfm_p_bar2p_d_dy_p_bar - function handle; derivative of 
    %       f_xfm_p_bar2p_d wrt y_p_bar
    %   p_p_inits - array; Nx2 array of initial guess of ideal points
    %   p_p_ds - array; Nx2 array of distorted points
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients corresponding to
    %       input symbolic function
    %   opts - struct;
    %       .xfm_p_d2p_it_cutoff - int; max number of iterations performed
    %           for inverse distortion refinement
    %       .xfm_p_d2p_norm_cutoff - scalar; cutoff for norm of difference
    %           of parameter vector for inverse distortion refinement
    % 
    % Outputs:
    %   p_ps - array; Nx2 array of optimized ideal points

    % Validate inputs
    util.validate_f_xfm_p_bar2p_d_args(f_xfm_p_bar2p_d);
    util.validate_f_xfm_p_bar2p_d_args(f_df_xfm_p_bar2p_d_dx_p_bar);  
    util.validate_f_xfm_p_bar2p_d_args(f_df_xfm_p_bar2p_d_dy_p_bar); 
    util.validate_A(A); 
    
    % Get camera matrix components
    alpha = A(1,1);
    x_o = A(1,3);
    y_o = A(2,3);   
    
    % Convert d to cell - used to pass distortion arguments into function
    % handle
    d_cell = num2cell(d);
    
    % Get initial x_p_bar and y_p_bar
    x_p_bar_inits = p_p_inits(:,1) - x_o;
    y_p_bar_inits = p_p_inits(:,2) - y_o;

    % Initialize parameter vector    
    num_points = size(p_p_inits,1);
    params = reshape([x_p_bar_inits y_p_bar_inits]',[],1);
    for it = 1:opts.xfm_p_d2p_it_cutoff
        % Get p_p_bar from p
        p_p_bars = reshape(params,2,[])';
        
        % Compute jacobian and residual
        dp_p_d_dx_p_bar = f_df_xfm_p_bar2p_d_dx_p_bar(p_p_bars(:,1),p_p_bars(:,2),alpha,x_o,y_o,d_cell{:});
        dp_p_d_dy_p_bar = f_df_xfm_p_bar2p_d_dy_p_bar(p_p_bars(:,1),p_p_bars(:,2),alpha,x_o,y_o,d_cell{:});
        
        % Jacobian - this is probably bottle neck
        jacob_cell = mat2cell(sparse(reshape(vertcat(dp_p_d_dx_p_bar',dp_p_d_dy_p_bar'),2,[])),2,2*ones(1,num_points));
        jacob = blkdiag(jacob_cell{:});
        
        % Residual 
        res = reshape(f_xfm_p_bar2p_d(p_p_bars(:,1),p_p_bars(:,2),alpha,x_o,y_o,d_cell{:})',[],1) - reshape(p_p_ds',[],1);

        % Get and store update
        delta_params = -lscov(jacob,res);
        params = params + delta_params;

        % Exit if change in distance is small
        if norm(delta_params) < opts.xfm_p_d2p_norm_cutoff
            break
        end
    end

    % Get p_p
    p_p_bars = reshape(params,2,[])';
    p_ps = p_p_bars + [x_o, y_o];
end