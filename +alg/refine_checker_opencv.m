function [p, cov] = refine_checker_opencv(array)
    % Performs "opencv" refinement of checker center on an array.
    %
    % Inputs:
    %   array - array; MxN array
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
    p = [(s+1)/2 (s+1)/2];    
    
    % Get gaussian mask
    kernel_gauss = mvnpdf([x y], p, cov);
    kernel_gauss = reshape(kernel_gauss,[s s]);

    % Get gradients and apply weights
    array_dx = alg.array_grad(array,'x');
    array_dy = alg.array_grad(array,'y');

    % Form linear system
    A = [array_dx(:) array_dy(:)];
    b = (array_dx(:).*x + array_dy(:).*y);

    % Solve for point and covariance
    [p,~,~,cov] = lscov(A,b,kernel_gauss(:).^2);
    p = p';
end