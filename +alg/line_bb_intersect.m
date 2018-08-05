function [p1, p2] = line_bb_intersect(l, bb)
    % finds intersection of line with bounding box
    %
    % Inputs:
    %   l - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    % 
    % Outputs:
    %   bb - array; 4x2 bounding box in the form:
    %       [top-left; bottom-left; top-right; bottom-right]
    
    % Get lines of bounding box
    l_bb = [1 0 -bb(1,1);
            1 0 -bb(2,1);
            0 1 -bb(1,2);
            0 1 -bb(2,2)];
    
    % Get intersections of line with bounding box lines
    p_bb = [line_line_intersect(l,l_bb(1,:));
            line_line_intersect(l,l_bb(2,:));
            line_line_intersect(l,l_bb(3,:));
            line_line_intersect(l,l_bb(4,:))];
    
    % Get two points which are on the bounding box
    bb_idx = find(((abs(p_bb(:,1) - bb(1,1)) < eps('single') | p_bb(:,1) > bb(1,1)) & (abs(p_bb(:,1) - bb(2,1)) < eps('single') | p_bb(:,1) < bb(2,1))) & ...
                  ((abs(p_bb(:,2) - bb(1,2)) < eps('single') | p_bb(:,2) > bb(1,2)) & (abs(p_bb(:,2) - bb(2,2)) < eps('single') | p_bb(:,2) < bb(2,2))));
    
    % TODO: handle degenerate cases
    if length(bb_idx) ~= 2
        error('More than two intersection points of bounding box found!');
    end
    
    % Return two intersection points
    p1 = p_bb(bb_idx(1),:);
    p2 = p_bb(bb_idx(2),:);
end

