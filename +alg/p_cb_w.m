function [p_cb_ws, p_fp_ws] = p_cb_w(opts)
    % This returns the calibration board points and the four point box 
    % on the calibration board in world coordinates.
    %
    % Inputs:
    %   opts - struct;
    %       .height_fp - scalar; height of the "four point" box
    %       .width_fp - scalar; width of the "four point" box
    %       .num_targets_height - int; number of targets in the "height" 
    %           dimension
    %       .num_targets_width - int; number of targets in the "width"
    %           dimension
    %       .target_spacing - scalar; space between targets
    %
    % Outputs:
    %   p_cb_ws - array; Nx2 array of calibration board points in world
    %       coordinates
    %   p_fp_ws - array; 4x2 array of the four point box in world
    %       coordinates
        
    % Get board points
    height_w = (opts.num_targets_height - 1)*opts.target_spacing;
    width_w = (opts.num_targets_width - 1)*opts.target_spacing;           
    [y_cb_ws, x_cb_ws] = ndgrid(0:opts.target_spacing:height_w, ...
                                0:opts.target_spacing:width_w);
    p_cb_ws = [x_cb_ws(:) y_cb_ws(:)];
                                                                
    % Get four point boxes; assume they are centered around board points
    p_fp_ws = [width_w/2 - opts.width_fp/2, height_w/2 - opts.height_fp/2;
               width_w/2 - opts.width_fp/2, height_w/2 + opts.height_fp/2;
               width_w/2 + opts.width_fp/2, height_w/2 - opts.height_fp/2;
               width_w/2 + opts.width_fp/2, height_w/2 + opts.height_fp/2];
end
