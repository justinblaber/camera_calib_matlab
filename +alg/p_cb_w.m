function p_cb_ws = p_cb_w(opts)
    % Returns calibration board world points
    %
    % Inputs:
    %   opts - struct;
    %       .num_targets_height - int; number of targets in the "height"
    %           dimension
    %       .num_targets_width - int; number of targets in the "width"
    %           dimension
    %       .target_spacing - scalar; space between targets
    %
    % Outputs:
    %   p_cb_ws - array; Nx2 array of calibration board world points

    % Get height and width
    h = (opts.num_targets_height-1)*opts.target_spacing;
    w = (opts.num_targets_width-1)*opts.target_spacing;

    % Get board points
    [y_cb_ws, x_cb_ws] = ndgrid(0:opts.target_spacing:h, ...
                                0:opts.target_spacing:w);
    p_cb_ws = [x_cb_ws(:) y_cb_ws(:)];
end
