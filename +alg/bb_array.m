function bb = bb_array(array)
    % Returns bounding box of array
    % 
    % Inputs:
    %   array - array; MxN array
    %    
    % Outputs:
    %   bb - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]

    h = size(array,1);
    w = size(array,2);
    
    bb = [1 1;
          w h];
end

