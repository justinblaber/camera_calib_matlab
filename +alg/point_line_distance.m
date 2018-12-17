function d = point_line_distance(p, l)
    % Computes closest distance between a point and line
    %
    % Inputs:
    %   p - array; 1x2 point
    %   l - array; 3x1 array of a line in the form:
    %       [a; b; c] where ax + by + c = 0
    %
    % Outputs:
    %   d - scalar; distance between point and line

    % Get parameters
    x = p(1);
    y = p(2);

    a = l(1);
    b = l(2);
    c = l(3);

    % Get distance
    d = abs(a*x + b*y + c)/sqrt(a^2 + b^2);
end
