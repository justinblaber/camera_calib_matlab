function [p1, p2] = line_bb_intersect(l, bb)
    % Finds the intersection(s) of a line with a bounding box
    %
    % Inputs:
    %   l - array; 3x1 array of a line in the form:
    %       [a; b; c] where ax + by + c = 0
    %   bb - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]
    %
    % Outputs:
    %   p1 - array; 1x2 point
    %   p2 - array; 1x2 point

    % Get lines of bounding box
    l_bbs = [1 0 -bb(1, 1);
             1 0 -bb(2, 1);
             0 1 -bb(1, 2);
             0 1 -bb(2, 2)];

    % Get intersections of line with bounding box lines
    p_bbs = [alg.line_line_intersect(l, l_bbs(1, :));
             alg.line_line_intersect(l, l_bbs(2, :));
             alg.line_line_intersect(l, l_bbs(3, :));
             alg.line_line_intersect(l, l_bbs(4, :))];

    % Get two points which are on the bounding box
    idx_bb = find(((abs(p_bbs(:, 1) - bb(1, 1)) < eps('single') | p_bbs(:, 1) > bb(1, 1)) & (abs(p_bbs(:, 1) - bb(2, 1)) < eps('single') | p_bbs(:, 1) < bb(2, 1))) & ...
                  ((abs(p_bbs(:, 2) - bb(1, 2)) < eps('single') | p_bbs(:, 2) > bb(1, 2)) & (abs(p_bbs(:, 2) - bb(2, 2)) < eps('single') | p_bbs(:, 2) < bb(2, 2))));

    % TODO: handle degenerate cases:
    %   1) line doesnt intersect bounding box          => no points
    %   2) line intersects only corner                 => single point
    %   3) line coincides with an edge of bounding box => infinite points

    if numel(idx_bb) ~= 2
        error(['Two intersection points of bounding box not found! ' ...
               'Please update function to handle degenerate cases.']);
    end

    % Return two intersection points
    p1 = p_bbs(idx_bb(1), :);
    p2 = p_bbs(idx_bb(2), :);
end
