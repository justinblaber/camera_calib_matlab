function p_imresize = p2imresize(p,sf)
    % Converts input points to coordinate system of imresize'd image.
    % 
    % Inputs:
    %   p - array; Nx2 array of points
    %   sf - scalar; scale factor used for resizing image with imresize()
    %    
    % Outputs:
    %   p_imresize - array; Nx2 array of points
    
    p_imresize = sf*(p - 1/2*(1-1/sf));
end