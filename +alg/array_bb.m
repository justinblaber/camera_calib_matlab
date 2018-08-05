function bb = array_bb(array)
    % Returns bounding box of array
    % 
    % Inputs:
    %   array - array; MxN array
    %    
    % Outputs:
    %   bb - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]

    height = size(array,1);
    width = size(array,2);
    
    bb = [0.5 0.5;
          width-0.5 height-0.5];
end

