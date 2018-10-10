function p_ps = p_p_d2p_p(p_p_ds,p_p_inits,f_p_p2p_p_d,f_dp_p_d_dx_p,f_dp_p_d_dy_p,A,d,opts)
    % Transforms points from distorted pixel coordinates to pixel 
    % coordinates.
    %
    % Inputs:
    %   p_p_ds - array; Nx2 array of distorted pixel points
    %   p_p_inits - array; Nx2 array of initial guess of pixel points
    %   f_p_p2p_p_d - function handle; describes the mapping between 
    %       pixel coordinates and distorted pixel coordinates.
    %   f_dp_p_d_dx_p - function handle; derivative of p_p2p_p_d wrt x_p
    %   f_dp_p_d_dy_p - function handle; derivative of p_p2p_p_d wrt y_p
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    %   opts - struct;
    %       .p_p_d2p_p_it_cutoff - int; max number of iterations performed
    %           for transformation from distorted pixel coordinates to 
    %           pixel coordinates.
    %       .p_p_d2p_p_norm_cutoff - scalar; cutoff for norm of difference
    %           of parameter vector for for transformation from distorted 
    %           pixel coordinates to pixel coordinates.
    % 
    % Outputs:
    %   p_ps - array; Nx2 array of pixel points
    
    % Initialize parameter vector    
    params = reshape(p_p_inits',[],1);
    for it = 1:opts.p_p_d2p_p_it_cutoff
        % Get p_ps from params
        p_ps = reshape(params,2,[])';
                                
        % Jacobian
        jacob = alg.dp_p_d_dp_p(p_ps,f_dp_p_d_dx_p,f_dp_p_d_dy_p,A,d);
        
        % Residual 
        res = reshape(alg.p_p2p_p_d(p_ps,f_p_p2p_p_d,A,d)',[],1) - reshape(p_p_ds',[],1);

        % Get and store update
        delta_params = -lscov(jacob,res);
        params = params + delta_params;

        % Exit if change in distance is small
        if norm(delta_params) < opts.p_p_d2p_p_norm_cutoff
            break
        end
    end

    % Get p_p
    p_ps = reshape(params,2,[])';
end