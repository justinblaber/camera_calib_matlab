function [y, x] = ndgrid_bb(bb)
    % Given a bounding box, this will return the ndgrid of it.
    %
    % Inputs:
    %   bb - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]
    %
    % Outputs:
    %   y - array; y coordinates returned by ndgrid
    %   x - array; x coordinates returned by ndgrid

    [y, x] = ndgrid(bb(1, 2):bb(2, 2), ...
                    bb(1, 1):bb(2, 1));
end
