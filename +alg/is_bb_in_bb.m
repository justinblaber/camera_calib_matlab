function success = is_bb_in_bb(bb1, bb2)
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
    %   success - logical; true if bounding box 1 is in bounding box 2
    
    % Initialize to false
    success = false;
    
    % Test in bounds
    if alg.is_p_in_bb(bb1(1,:),bb2) && alg.is_p_in_bb(bb1(2,:),bb2)
        success = true;
    end   
end