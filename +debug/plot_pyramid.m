function plot_pyramid(pyramid,a)
    % Plots an image pyramid

    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
        
    % Get width and height of composite image    
    height_total = 0;             
    width_total = 0;                    
    for i = 1:length(pyramid)
        height_total = height_total + size(pyramid{i},1);
        width_total = width_total + size(pyramid{i},2);
    end

    % Fill in image
    pyramid_composite = zeros(height_total,width_total);   
    x_idx = 1;
    for i = 1:length(pyramid)                
        for j = 1:size(pyramid{i},3)
            y_idx = (j-1)*size(pyramid{i},1)+1;
            pyramid_composite(y_idx:y_idx+size(pyramid{i},1)-1, ...
                              x_idx:x_idx+size(pyramid{i},2)-1) = pyramid{i}(:,:,j);
        end
        x_idx = x_idx + size(pyramid{i},2); 
    end 

    % Display
    imshow(pyramid_composite,[],'parent',a);  
end