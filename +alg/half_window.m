function hw = half_window(point_w,homography,window_factor,cb_config)
    % Computes half window used for refinement window
    %   
    % Inputs:
    %   point_w - array; 1x2 center point of window in world coordinates    
    %   homography - array; 3x3 homography
    %   window_factor - scalar; proportion of square_size used to compute
    %       window
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   hw - scalar; window = 2*half_window+1
    
    hw = floor(max(alg.window_lengths_i(point_w,homography,window_factor,cb_config))/4)*2+1;
end