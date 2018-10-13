function [p, cov_p] = refine_checker_edges(array_dx,array_dy,l1,l2,opts)
    % Performs "edges" refinement of a checker center.
    %
    % Inputs:
    %   array_dx - array; MxM array gradient in x direction
    %   array_dy - array; MxM array gradient in y direction
    %   l1 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    %   l2 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    %   opts - struct;
    %       .refine_checker_edges_h2_init - scalar; initial value of h2
    %           parameter in "edges" checker refinement
    %       .refine_checker_edges_it_cutoff - int; max number of 
    %           iterations performed for "edges" checker refinement 
    %       .refine_checker_edges_norm_cutoff - scalar; cutoff for the 
    %           difference in norm of the parameter vector for "edges" 
    %           checker refinement
    %
    % Outputs:
    %   p - array; 1x2 refined checker center
    %   cov_p - array; 2x2 covariance array
        
    if size(array_dx,1) ~= size(array_dx,2) || ~isequal(size(array_dx),size(array_dy))
        error('Input gradient arrays must be square and equal in size');
    end
    
    % Get size
    s = length(array_dx);
    
    % Get coordinates of pixels
    [ys,xs] = ndgrid(1:s,1:s);
    xs = xs(:);
    ys = ys(:);
    
    % Get covariance matrix of gaussian kernel
    sigma = (s-1)/4;
    cov = [sigma^2 0; ...
           0       sigma^2];
    
    % Initial guess for center point
    p = alg.line_line_intersect(l1, l2);    
            
    % Get gradient magnitude - I found that using squared magnitude is
    % better because it tends to supress smaller gradients due to noise
    array_grad_mag = array_dx.^2 + array_dy.^2;
    
    % Normalize gradient magnitude between 0 and 1
    array_grad_mag = (array_grad_mag-min(array_grad_mag(:)))./(max(array_grad_mag(:))-min(array_grad_mag(:)));
        
    % Create initial parameter vector
    params = [1;
              opts.refine_checker_edges_h2_init;
              atan(-l1(1)/l1(2));
              atan(-l2(1)/l2(2));
              p(1);
              p(2)];
    
    % Perform iterations until convergence
    for it = 1:opts.refine_checker_edges_it_cutoff  
        % Get gauss newton parameters
        [hess, grad] = get_gauss_newton_params(params, ...
                                               array_grad_mag, ...
                                               xs, ...
                                               ys, ...
                                               cov);

        % Get and store update
        delta_params = -lscov(hess,grad);
        params = params + delta_params;        
         
        % Exit if change in distance is small
        if norm(delta_params) < opts.refine_checker_edges_norm_cutoff
            break
        end        
    end
        
    % Get center point
    p = params(5:6)';    

    % Get covariance of center point
    [hess, ~, res] = get_gauss_newton_params(params, ...
                                             array_grad_mag, ...
                                             xs, ...
                                             ys, ...
                                             cov);
    mse = res'*res/(numel(res)-numel(params));
    cov_p = inv(hess)*mse; %#ok<MINV>
    cov_p = cov_p(5:6,5:6);
end

function [hess, grad, res] = get_gauss_newton_params(params,array_grad_mag,xs,ys,cov)
    % Get center point
    p = params(5:6)';

    % Get gaussian mask
    kernel_gauss = mvnpdf([xs ys], p, cov);
    kernel_gauss = reshape(kernel_gauss,size(array_grad_mag));

    % Sample edge function        
    f = params(1)*exp(-params(2)^2*((xs-params(5))*sin(params(3))-(ys-params(6))*cos(params(3))).^2) + ...
        params(1)*exp(-params(2)^2*((xs-params(5))*sin(params(4))-(ys-params(6))*cos(params(4))).^2) - ...
        2*params(1)*exp(-params(2)^2*((xs-params(5)).^2+(ys-params(6)).^2)); 

    % Get residuals
    res = f-array_grad_mag(:);
    res = res.*kernel_gauss(:); % Apply weights

    % Get gradient of edge function
    df_dparams = [(exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2) - 2*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)) + exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2))';
                  (4*params(1)*params(2)*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)).*((params(5) - xs).^2 + (params(6) - ys).^2) - 2*params(1)*params(2)*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2 - 2*params(1)*params(2)*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2)';
                  (2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(cos(params(3))*(params(5) - xs) + sin(params(3))*(params(6) - ys)).*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)))';
                  (2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(cos(params(4))*(params(5) - xs) + sin(params(4))*(params(6) - ys)).*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)))';
                  (2*params(1)*params(2)^2*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)).*(2*params(5) - 2*xs) + 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(sin(params(3))*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs))) + 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(sin(params(4))*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs))))';
                  (2*params(1)*params(2)^2*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)).*(2*params(6) - 2*ys) - 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(cos(params(3))*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs))) - 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(cos(params(4))*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs))))'];
    df_dparams = df_dparams.*kernel_gauss(:)'; % Apply weights

    % Get cost function gradient and hessian
    grad = 2*sum(res'.*df_dparams,2);
    hess = 2*(df_dparams*df_dparams');
end
