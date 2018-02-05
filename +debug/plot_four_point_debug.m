function plot_four_point_debug(four_points_p,four_points_debug,cb_img,calib_config,a)
    % This plots residuals
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Show image    
    array = cb_img.get_gs();
    if calib_config.four_point_detect_scaled_array_min_size == realmax
        scale_factor = 1;
    else
        scale_factor = calib_config.four_point_detect_scaled_array_min_size/min(size(array));
        array = imresize(array,scale_factor);
    end
    imshow(array,[],'Parent',a)
    hold(a,'on');
    
    % Must rescale four_points_ps
    plot(scale_factor*(four_points_p(:,1)-1/2*(1-1/scale_factor)), ...
         scale_factor*(four_points_p(:,2)-1/2*(1-1/scale_factor)),'-mo','MarkerSize',8, ...
         'parent',a);
     
    % Debugging stuff does not need to be rescaled
    axes(a);
    for i = 1:length(four_points_debug.blobs)
        external.ellipse(four_points_debug.blobs(i).r, ...
                         four_points_debug.blobs(i).r, ...
                         0, ...
                         four_points_debug.blobs(i).x, ...
                         four_points_debug.blobs(i).y, ...
                         'r');  
    end
    for i = 1:length(four_points_debug.ellipses)
        external.ellipse(four_points_debug.ellipses(i).r1, ...
                         four_points_debug.ellipses(i).r2, ...
                         four_points_debug.ellipses(i).rot, ...
                         four_points_debug.ellipses(i).x, ...
                         four_points_debug.ellipses(i).y, ...
                         'g');  
    end
    
    % Remove hold
    hold(a,'off');
end