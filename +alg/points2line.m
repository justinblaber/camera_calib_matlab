function l = points2line(p1,p2)
    % Computes line given two points.
    % 
    % Inputs:
    %   p1 - array; 1x2 point 
    %   p2 - array; 1x2 point 
    %    
    % Outputs:
    %   l - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0

    % Get x and y coordinates of points
    x1 = p1(1);
    y1 = p1(2);
    
    x2 = p2(1);
    y2 = p2(2);
    
    % Get slope
    m = (y2-y1)/(x2-x1);
    
    % Get line
    l = alg.pointslope2line(p1,m);
end