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
    [y,x] = ndgrid(1:s,1:s);
    x = x(:);
    y = y(:);
    
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
                                               x, ...
                                               y, ...
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
    [hess, ~, r] = get_gauss_newton_params(params, ...
                                           array_grad_mag, ...
                                           x, ...
                                           y, ...
                                           cov);
    mse = r'*r/(numel(r)-numel(params));
    cov_p = inv(hess)*mse; %#ok<MINV>
    cov_p = cov_p(5:6,5:6);
end

function [hess, grad, r] = get_gauss_newton_params(h,array_grad_mag,x,y,cov)
    % Get center point
    p = h(5:6)';

    % Get gaussian mask
    kernel_gauss = mvnpdf([x y], p, cov);
    kernel_gauss = reshape(kernel_gauss,size(array_grad_mag));

    % Sample edge function        
    f = h(1)*exp(-h(2)^2*((x-h(5))*sin(h(3))-(y-h(6))*cos(h(3))).^2) + ...
        h(1)*exp(-h(2)^2*((x-h(5))*sin(h(4))-(y-h(6))*cos(h(4))).^2) - ...
        2*h(1)*exp(-h(2)^2*((x-h(5)).^2+(y-h(6)).^2)); 

    % Get residuals
    r = f-array_grad_mag(:);
    r = r.*kernel_gauss(:); % Apply weights

    % Get gradient of edge function
    df_dh = [(exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2) - 2*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)) + exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2))';
             (4*h(1)*h(2)*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)).*((h(5) - x).^2 + (h(6) - y).^2) - 2*h(1)*h(2)*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2 - 2*h(1)*h(2)*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2)';
             (2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(cos(h(3))*(h(5) - x) + sin(h(3))*(h(6) - y)).*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)))';
             (2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(cos(h(4))*(h(5) - x) + sin(h(4))*(h(6) - y)).*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)))';
             (2*h(1)*h(2)^2*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)).*(2*h(5) - 2*x) + 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(sin(h(3))*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x))) + 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(sin(h(4))*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x))))';
             (2*h(1)*h(2)^2*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)).*(2*h(6) - 2*y) - 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(cos(h(3))*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x))) - 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(cos(h(4))*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x))))'];
    df_dh = df_dh.*kernel_gauss(:)'; % Apply weights

    % Get cost function gradient and hessian
    grad = 2*sum(r'.*df_dh,2);
    hess = 2*(df_dh*df_dh');
end
