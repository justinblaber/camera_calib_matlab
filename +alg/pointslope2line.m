function l = pointslope2line(p,m)
    % Computes a line given a point and slope
    % 
    % Inputs:
    %   p - array; 1x2 point 
    %   m - scalar; slope
    %    
    % Outputs:
    %   l - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0

    % Get x and y coordinates of points
    x = p(1);
    y = p(2);
    
    % Form homogeneous line
    if isinf(m)
        a = 1;
        b = 0;
        c = -x;
    else
        a = m;
        b = -1;
        c = y-m*x;
    end
    
    % Store
    l = [a b c];
end
