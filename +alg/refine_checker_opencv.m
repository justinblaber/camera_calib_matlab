function [p, cov_p] = refine_checker_opencv(array_dx,array_dy,p)
    % Performs "opencv" refinement of a checker center on input array
    % gradients.
    %
    % Inputs:
    %   array_dx - array; MxN array gradient in x direction
    %   array_dy - array; MxN array gradient in y direction
    %   p - array; 1x2 initial guess of checker center
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
        
    % Get gaussian mask
    kernel_gauss = mvnpdf([x y], p, cov);
    kernel_gauss = reshape(kernel_gauss,[s s]);

    % Form linear system
    A = [array_dx(:) array_dy(:)];
    b = array_dx(:).*x + array_dy(:).*y;

    % Solve for center point and center point covariance using gaussian
    % kernel as weights
    [p,~,~,cov_p] = lscov(A,b,kernel_gauss(:).^2);
    p = p';
end
