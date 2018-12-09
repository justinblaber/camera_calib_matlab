function [p, cov_p] = refine_checker_opencv(array_dx,array_dy,p)
    % Performs "opencv" refinement of a checker center on input array
    % gradients.
    %
    % Inputs:
    %   array_dx - array; MxM array gradient in x direction
    %   array_dy - array; MxM array gradient in y direction
    %   p - array; 1x2 initial guess of checker center
    %
    % Outputs:
    %   p - array; 1x2 refined checker center
    %   cov_p - array; 2x2 covariance of checker center
    
    % Make sure gradients are equal in size and square
    if size(array_dx,1) ~= size(array_dx,2) || ~isequal(size(array_dx),size(array_dy))
        error('Input gradient arrays must be square and equal in size');
    end
    
    % Get size and bounding box
    s = size(array_dx,1);
    bb_array = alg.bb_array(array_dx);
    
    % Initialize weights
    W_init = double(~isnan(array_dx) & ~isnan(array_dy));
    
    % Remove any NaNs from array gradient
    array_dx(isnan(array_dx)) = 0;
    array_dy(isnan(array_dy)) = 0;
        
    % Get coordinates of pixels
    [ys,xs] = alg.ndgrid_bb(bb_array);
    xs = xs(:);
    ys = ys(:);
    
    % Get gaussian kernel
    sigma_gauss = (s-1)/4;
    cov_gauss = [sigma_gauss^2 0; ...
                 0             sigma_gauss^2];
    kernel_gauss = mvnpdf([xs ys], p, cov_gauss);
    kernel_gauss = reshape(kernel_gauss,[s s]);
    kernel_gauss = alg.normalize_array(kernel_gauss,'min-max');
    
    % Update weights
    W = kernel_gauss.*W_init;
    
    % Form linear system
    A = [array_dx(:) array_dy(:)];
    b = array_dx(:).*xs + array_dy(:).*ys;
        
    % Solve for center point and center point covariance using gaussian
    % kernel as weights
    [p,~,~,cov_p] = lscov(A,b,W(:));
    p = p';
end