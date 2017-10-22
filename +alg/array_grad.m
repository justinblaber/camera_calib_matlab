function grad = array_grad(array,direc)
    % Computes gradient of input array along the direction specified by 
    % direc
    %
    % Inputs:
    %   array - array; MxN array
    %   direc - string; either 'x' or 'y'
    %
    % Outputs:
    %   grad - array; MxN gradient component along either x or y direction
    
    switch direc
        case 'x'
            grad = imfilter(array,-fspecial('sobel')','same','replicate');
        case 'y'
            grad = imfilter(array,-fspecial('sobel'),'same','replicate');
        otherwise 
            error(['Direction of gradient calculation can either be: x or y' ...
                   ', but : ' direc ' was input']);
    end
end