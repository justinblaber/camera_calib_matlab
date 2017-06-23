function array_g = array_grad(array,direc)
    % Compute gradient of input image along direction specified by direc
    %
    % Inputs:
    %   array - array; mxn array
    %   direc - string; either 'x' or 'y'
    %
    % Outputs:
    %   array_g - array; mxn gradient
    
    switch direc
        case 'x'
            array_g = imfilter(array,-fspecial('sobel')');
        case 'y'
            array_g = imfilter(array,-fspecial('sobel'));
        otherwise 
            error(['Direction of gradient calculation can either be: x or y' ...
                   ', but : ' direc ' was input']);
    end
end