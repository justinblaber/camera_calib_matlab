function array = gauss_array(array,sigma)
    % Applies gaussian kernel to input array at specified sigma.
    %
    % Inputs:
    %   array - array; MxN array
    %   sigma - scalar; standard deviation of gaussian distribution
    %
    % Outputs:
    %   array - array; MxN gaussian filtered array
    
    % window must be odd so kernel is centered; this ensures it.
    window = 2*ceil(3*sigma)+1;
    array = imfilter(array,fspecial('gaussian',[window window],sigma), ...
                     'same','replicate');
end
