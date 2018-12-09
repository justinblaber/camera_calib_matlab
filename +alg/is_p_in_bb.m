function in_bb = is_p_in_bb(p, bb)
    % Given a point and a bounding box, this will return true if p is 
    % inside of it, and false otherwise.
    % 
    % Inputs:
    %   p - array; 1x2 point 
    %   bb - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]
    % 
    % Outputs:
    %   in_bb - logical; true if point is in bounding box
    
    % Initialize to false
    in_bb = false;
    
    % Test in bounds
    if p(1) >= bb(1,1) && p(2) >= bb(1,2) && ...
       p(1) <= bb(2,1) && p(2) <= bb(2,2)
        in_bb = true;
    end   
end