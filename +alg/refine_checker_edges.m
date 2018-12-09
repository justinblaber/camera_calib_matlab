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
    
    % Initialize weights
    W_init = double(~isnan(array_dx) & ~isnan(array_dy));
    
    % Remove any NaNs from array gradient
    array_dx(isnan(array_dx)) = 0;
    array_dy(isnan(array_dy)) = 0;
    
    % Get size and bounding box
    s = size(array_dx,1);
    bb_array = alg.bb_array(array_dx);
    
    % Get coordinates of pixels
    [ys,xs] = alg.ndgrid_bb(bb_array);
    xs = xs(:);
    ys = ys(:);
    
    % Set gaussian kernel parameters
    sigma_gauss = (s-1)/4;
    cov_gauss = [sigma_gauss^2 0; ...
                 0             sigma_gauss^2];
    
    % Initial guess for center point
    p_init = alg.line_line_intersect(l1, l2);    
            
    % Get gradient magnitude - I found that using squared magnitude is
    % better because it tends to supress smaller gradients due to noise
    array_grad_mag = array_dx.^2 + array_dy.^2;
    
    % Normalize gradient magnitude between 0 and 1
    array_grad_mag = alg.normalize_array(array_grad_mag,'min-max');
        
    % Create initial parameter vector
    params = [1;
              opts.refine_checker_edges_h2_init;
              atan(-l1(1)/l1(2));
              atan(-l2(1)/l2(2));
              p_init(1);
              p_init(2)];
    
    % Perform iterations until convergence
    for it = 1:opts.refine_checker_edges_it_cutoff  
        % Get gauss newton parameters
        [jacob, res] = calc_gauss_newton_params(params, ...
                                                array_grad_mag, ...
                                                xs, ...
                                                ys);
                                            
        % Update weights
        kernel_gauss = mvnpdf([xs ys], params(5:6)', cov_gauss);
        kernel_gauss = reshape(kernel_gauss,[s s]);
        kernel_gauss = alg.normalize_array(kernel_gauss,'min-max');
        W = kernel_gauss.*W_init;

        % Get and store update
        delta_params = -lscov(jacob,res,W(:));
        params = params + delta_params;
        
        % Make sure point doesnt go outside of bounding box
        if ~alg.is_p_in_bb(params(5:6)',bb_array)
            p = nan(1,2);
            cov_p = nan(2);
            return
        end
         
        % Exit if change in distance is small
        if norm(delta_params) < opts.refine_checker_edges_norm_cutoff
            break
        end        
    end
        
    % Get center point
    p = params(5:6)';    
    
    % Get covariance of center point
    [jacob, res] = calc_gauss_newton_params(params, ...
                                            array_grad_mag, ...
                                            xs, ...
                                            ys);
                             
    % Update weights
    kernel_gauss = mvnpdf([xs ys], params(5:6)', cov_gauss);
    kernel_gauss = reshape(kernel_gauss,[s s]);
    kernel_gauss = alg.normalize_array(kernel_gauss,'min-max');
    W = kernel_gauss.*W_init;
    
    % Get covaraince
    [~,~,~,cov_params] = lscov(jacob,res,W(:));
    cov_p = cov_params(5:6,5:6);
end

function [jacob, res] = calc_gauss_newton_params(params,array_grad_mag,xs,ys)
    % Sample edge function        
    f = params(1)*exp(-params(2)^2*((xs-params(5))*sin(params(3))-(ys-params(6))*cos(params(3))).^2) + ...
        params(1)*exp(-params(2)^2*((xs-params(5))*sin(params(4))-(ys-params(6))*cos(params(4))).^2) - ...
        2*params(1)*exp(-params(2)^2*((xs-params(5)).^2+(ys-params(6)).^2)); 

    % Get residuals
    res = f-array_grad_mag(:);

    % Get jacobian of edge function
    jacob = [exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2) - 2*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)) + exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2), ...
             4*params(1)*params(2)*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)).*((params(5) - xs).^2 + (params(6) - ys).^2) - 2*params(1)*params(2)*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2 - 2*params(1)*params(2)*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2, ...
             2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(cos(params(3))*(params(5) - xs) + sin(params(3))*(params(6) - ys)).*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)), ...
             2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(cos(params(4))*(params(5) - xs) + sin(params(4))*(params(6) - ys)).*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)), ...
             2*params(1)*params(2)^2*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)).*(2*params(5) - 2*xs) + 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(sin(params(3))*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs))) + 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(sin(params(4))*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs))), ...
             2*params(1)*params(2)^2*exp(-params(2)^2*((params(5) - xs).^2 + (params(6) - ys).^2)).*(2*params(6) - 2*ys) - 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs)).^2).*(cos(params(3))*(cos(params(3))*(params(6) - ys) - sin(params(3))*(params(5) - xs))) - 2*params(1)*params(2)^2*exp(-params(2)^2*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)).^2).*(cos(params(4))*(cos(params(4))*(params(6) - ys) - sin(params(4))*(params(5) - xs)))];
end
