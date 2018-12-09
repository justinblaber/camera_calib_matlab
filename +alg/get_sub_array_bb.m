function sub_array = get_sub_array_bb(array,bb)
    % Given an array and a bounding box, this will grab the sub array
    %
    % Inputs:
    %   array - array; MxN array
    %   bb - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]
    % 
    % Outputs:
    %   sub_array - array; sub array specified by bounding box
    
    % Grab sub array
    sub_array = array(bb(1,2):bb(2,2), ...
                      bb(1,1):bb(2,1)); 
end