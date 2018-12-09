function in_bb = is_bb_in_bb(bb1, bb2)
    % Given bounding box 1 and bounding box 2, this will return true if
    % bounding box 1 is contained in bounding box 2
    % 
    % Inputs:
    %   bb1 - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]
    %   bb2 - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]
    % 
    % Outputs:
    %   in_bb - logical; true if bounding box 1 is in bounding box 2
    
    % Initialize to false
    in_bb = false;
    
    % Test in bounds
    if bb1(1,1) >= bb2(1,1) && bb1(1,2) >= bb2(1,2) && ...
       bb1(2,1) <= bb2(2,1) && bb1(2,2) <= bb2(2,2)
        in_bb = true;
    end   
end