function p = refine_checker(array_dx,array_dy,opts)
    
    if size(array_dx,1) ~= size(array_dx,2) || ~isequal(size(array_dx),size(array_dy))
        error('Gradient arrays must be square and have equal size');
    end

    % Get size
    s = size(array_dx,1);
    
    % Get coordinates of pixels
    [y,x] = ndgrid(1:s,1:s);
    
    % Get covariance matrix of gaussian kernel
    sigma = s/8;
    cov = [sigma^2 0; ...
           0       sigma^2];
    
    % Initial guess for center point
    p = [(s+1)/2 (s+1)/2];    
    
    % Iterate
    for i = 1:opts.refine_checker_it_cutoff
        % Keep copy of point before updating it        
        p_prev = p; 

        % Get gaussian mask
        kernel_gauss = mvnpdf([x(:) y(:)], p, cov);
        kernel_gauss = reshape(kernel_gauss,[s s]);
        
        % Apply weights
        array_dx_w = array_dx.*kernel_gauss;
        array_dy_w = array_dy.*kernel_gauss;

        % Form linear system
        A = [array_dx_w(:) array_dy_w(:)];
        b = (array_dx_w(:).*x(:) + array_dy_w(:).*y(:));

        % Solve for updated point
        p = mldivide(A,b)';  
        
        % Exit if change in distance is small
        diff_norm = norm(p-p_prev);
        if diff_norm < opts.refine_checker_norm_cutoff
            break
        end
    end
end