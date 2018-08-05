function [board_points_w, four_points_w] = cb_points(opts)
    % This will return the calibration board points in world coordinates
    % and the four point box in world coordinates.
    %
    % Inputs:
    %   opts - struct;
    %       .four_point_height - scalar; height of the "four point" box
    %       .four_point_width - scalar; width of the "four point" box
    %       .num_targets_height - int; number of targets in the "height" 
    %           dimension
    %       .num_targets_width - int; number of targets in the "width"
    %           dimension
    %       .target_spacing - scalar; space between targets
    %
    % Outputs:
    %   board_points_w - cell; MxN cell array of calibration board points
    %       in world coordinates
    %   four_points_w - cell; 2x2 cell array of the four point box in world
    %       coordinates
        
    % Get board points
    t_h = (opts.num_targets_height-1) * opts.target_spacing;
    t_w = (opts.num_targets_width-1) * opts.target_spacing;           
    [board_y, board_x] = ndgrid(0:opts.target_spacing:t_h, ...
                                0:opts.target_spacing:t_w);
    board_points_w = [board_x(:) board_y(:)];
                                                                
    % Get four points - assume they are centered around board points
    fp_h = opts.four_point_height;
    fp_w = opts.four_point_width;
    four_points_w = [t_w/2-fp_w/2 t_h/2-fp_h/2;
                     t_w/2-fp_w/2 t_h/2+fp_h/2;
                     t_w/2+fp_w/2 t_h/2-fp_h/2;
                     t_w/2+fp_w/2 t_h/2+fp_h/2];
end
