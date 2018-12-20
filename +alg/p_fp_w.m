function p_fp_ws = p_fp_w(opts)
    % Returns the four point box on the calibration board in world
    % coordinates.
    %
    % Inputs:
    %   opts - struct;
    %       .num_targets_height - int; number of targets in the "height"
    %           dimension
    %       .num_targets_width - int; number of targets in the "width"
    %           dimension
    %       .target_spacing - scalar; space between targets
    %       .height_fp - scalar; height of the "four point" box
    %       .width_fp - scalar; width of the "four point" box
    %
    % Outputs:
    %   p_fp_ws - array; 4x2 array of the four point box in world
    %       coordinates

    % Get height and width
    h = (opts.num_targets_height-1)*opts.target_spacing;
    w = (opts.num_targets_width-1)*opts.target_spacing;

    % Get four point box; assume it is centered on the calibration board
    p_fp_ws = [w/2 - opts.width_fp/2, h/2 - opts.height_fp/2;
               w/2 - opts.width_fp/2, h/2 + opts.height_fp/2;
               w/2 + opts.width_fp/2, h/2 - opts.height_fp/2;
               w/2 + opts.width_fp/2, h/2 + opts.height_fp/2];
end
