function array_g = array_gauss(array,sigma)
    % Applies gaussian kernel to input array at specified sigma.
    %
    % Inputs:
    %   array - array; MxN array
    %   sigma - scalar; standard deviation of gaussian distribution
    %
    % Outputs:
    %   array_g - array; MxN gaussian filtered array
    
    % window must be odd so kernel is centered; this ensures it.
    window = 6*ceil(sigma)+1;
    array_g = imfilter(array,fspecial('gaussian',[window window],sigma), ...
                       'same','replicate');
end