function plot_blobs(blobs,array,a)
    % Plots blobs

    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Plot background image
    imshow(array,[],'parent',a);
    hold(a,'on');
    
    % Plot blobs    
    for i = 1:length(blobs)                
        % ellipse does not have an input argument for axes unfortunately
        util.ellipse(blobs(i).r, ...
                     blobs(i).r, ...
                     0, ...
                     blobs(i).x, ...
                     blobs(i).y, ...
                     'r');
    end
    
    % Remove hold
    drawnow
    hold(a,'off');
end