function p = imresize2p(p_imresize, sf)
    % Converts input points in imresize'd image to original coordinate
    % system.
    %
    % Inputs:
    %   p_imresize - array; Nx2 array of points
    %   sf - scalar; scale factor used for resizing image with imresize()
    %
    % Outputs:
    %   p - array; Nx2 array of points

    p = 1/sf*(p_imresize + 1/2*(sf-1));
end
