function [p, cov] = refine_checker_edges(array,l1,l2,opts)
    % Performs "edges" refinement of checker center on an array.
    %
    % Inputs:
    %   array - array; MxN array
    %   l1 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    %   l2 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    %   opts - struct;
    %       .refine_checker_edges_h1_init - scalar; initial value of h1
    %           parameter in "edges" checker refinement
    %       .refine_checker_edges_h2_init -  scalar; initial value of h2
    %           parameter in "edges" checker refinement
    %       .refine_checker_edges_it_cutoff - int; max number of 
    %           iterations performed for "edges" checker refinement 
    %       .refine_checker_edges_norm_cutoff  - scalar; cutoff for the 
    %           difference in norm of the parameter vector for "edges" 
    %           checker refinement
    %
    % Outputs:
    %   p - array; 1x2 sub pixel point at center of checker
    %   cov - array; 2x2 covariance array
        
    if size(array,1) ~= size(array,2)
        error('Input array must be square');
    end

    % Normalize array so gradients aren't crazy
    array = (array-min(array(:)))./(max(array(:))-min(array(:)));
    
    % Get size
    s = size(array,1);
    
    % Get coordinates of pixels
    [y,x] = ndgrid(1:s,1:s);
    x = x(:);
    y = y(:);
    
    % Get covariance matrix of gaussian kernel
    sigma = s/4;
    cov = [sigma^2 0; ...
           0       sigma^2];
    
    % Initial guess for center point
    p = alg.line_line_intersect(l1, l2);    
    
    % Get gaussian mask
    kernel_gauss = mvnpdf([x y], p, cov);
    kernel_gauss = reshape(kernel_gauss,[s s]);

    % Get gradients
    array_dx = alg.array_grad(array,'x');
    array_dy = alg.array_grad(array,'y');
    
    % Get gradient magnitude
    array_grad_mag = sqrt(array_dx.^2 + array_dy.^2);
        
    % Create initial parameter vector
    h = [opts.refine_checker_edges_h1_init;
         opts.refine_checker_edges_h2_init;
    	 atan(-l1(1)/l1(2));
         atan(-l2(1)/l2(2));
         p(1);
         p(2)];
    
    for it = 1:opts.refine_checker_edges_it_cutoff
        % Sample edge function        
        f = h(1)*exp(-h(2)^2*((x-h(5))*sin(h(3))-(y-h(6))*cos(h(3))).^2) + ...
            h(1)*exp(-h(2)^2*((x-h(5))*sin(h(4))-(y-h(6))*cos(h(4))).^2) - ...
            2*h(1)*exp(-h(2)^2*((x-h(5)).^2+(y-h(6)).^2)); 
        
        % Get residuals
        r = f-array_grad_mag(:);
        r = r.*kernel_gauss(:);
        
        % Get gradient of edge function
        df_dh = [(exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2) - 2*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)) + exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2))';
                 (4*h(1)*h(2)*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)).*((h(5) - x).^2 + (h(6) - y).^2) - 2*h(1)*h(2)*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2 - 2*h(1)*h(2)*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2)';
                 (2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(cos(h(3))*(h(5) - x) + sin(h(3))*(h(6) - y)).*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)))';
                 (2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(cos(h(4))*(h(5) - x) + sin(h(4))*(h(6) - y)).*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)))';
                 (2*h(1)*h(2)^2*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)).*(2*h(5) - 2*x) + 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(sin(h(3))*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x))) + 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(sin(h(4))*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x))))';
                 (2*h(1)*h(2)^2*exp(-h(2)^2*((h(5) - x).^2 + (h(6) - y).^2)).*(2*h(6) - 2*y) - 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x)).^2).*(cos(h(3))*(cos(h(3))*(h(6) - y) - sin(h(3))*(h(5) - x))) - 2*h(1)*h(2)^2*exp(-h(2)^2*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x)).^2).*(cos(h(4))*(cos(h(4))*(h(6) - y) - sin(h(4))*(h(5) - x))))'];
        df_dh = df_dh.*kernel_gauss(:)';
             
        % Get cost function gradient and hessian
        grad = 2*sum(r'.*df_dh,2);
        hess = 2*(df_dh*df_dh');

        % Get and store update
        delta_h = -inv(hess)*grad;
        h = h + delta_h;
         
        % Exit if change in distance is small
        if norm(delta_h) < opts.refine_checker_edges_norm_cutoff
            break
        end
    end
    
    % Store final center point
    p = h(5:6)';    
    
    % Compute covariance
    mse = r'*r/(numel(r)-numel(h));
    cov = inv(hess)*mse; %#ok<MINV>
    cov = cov(5:6,5:6);
end