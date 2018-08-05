function p = line_line_intersect(l1,l2)
    % Computes intersection of two lines
    % 
    % Inputs:
    %   l1 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    %   l2 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    %    
    % Outputs:
    %   p - array; 1x2 point 

    a1 = l1(1);
    b1 = l1(2);
    c1 = l1(3);
    
    a2 = l2(1);
    b2 = l2(2);
    c2 = l2(3);
    
    % TODO: handle degenerate case of a1*b2 - b1*a2 = 0, or just let it be
    % Inf/NaN?

    p = [(-c1*b2 + b1*c2)/(a1*b2 - b1*a2) ...
         (-a1*c2 + c1*a2)/(a1*b2 - b1*a2)];
end

