function rw = refine_window(four_points_i,cb_config)
    % This will return the refinement window based on the input four points
    % and cb_config.
    %
    % Inputs:
    %   four_points_i - array; 4x2 array of four points. The four corners 
    %       of the calibration board in image space.
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   rw - refinement window parameter (note that width of refinement 
    %       window will be 2*rw+1)
    
    % TODO: instead of having a fixed window per calibration board,
    % possibly decide on the window on a per point basis.
    
    % Get calibration board points
    [board_points_w, four_points_w] = alg.cb_points(cb_config);
    
    % Compute homography    
    homography = alg.linear_homography(four_points_w,four_points_i);
    
    % Apply homography to board_points
    board_points_i = alg.apply_homography(homography,board_points_w);
    
    % Compute distances from points adjacent to four corners. The minimum
    % of this should be the minimium distance between any two corners on
    % the calibration board.
    h = cb_config.num_rects_height;
    w = cb_config.num_rects_width;
    % 1st corner
    dists(1) = norm(board_points_i(1,:) - board_points_i(2,:));
    dists(2) = norm(board_points_i(1,:) - board_points_i(2+h,:));
    dists(3) = norm(board_points_i(1,:) - board_points_i(3+h,:));
    % 2nd corner
    dists(4) = norm(board_points_i(1+h,:) - board_points_i(h,:));
    dists(5) = norm(board_points_i(1+h,:) - board_points_i(1+2*h,:));
    dists(6) = norm(board_points_i(1+h,:) - board_points_i(2+2*h,:));
    % 3rd corner
    dists(7) = norm(board_points_i((1+h)*w+1,:) - board_points_i((1+h)*(w-1)+1,:));
    dists(8) = norm(board_points_i((1+h)*w+1,:) - board_points_i((1+h)*(w-1)+2,:));
    dists(9) = norm(board_points_i((1+h)*w+1,:) - board_points_i((1+h)*w+2,:));
    % 4th corner
    dists(10) = norm(board_points_i((1+h)*(1+w),:) - board_points_i((1+h)*w,:));
    dists(11) = norm(board_points_i((1+h)*(1+w),:) - board_points_i((1+h)*w-1,:));
    dists(12) = norm(board_points_i((1+h)*(1+w),:) - board_points_i((1+h)*(1+w)-1,:));

    % Set refine_window using minimum of this distance
    rw = ceil(min(dists)/4*sqrt(2));
end