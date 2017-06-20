function debug_cb_points(points,cb_img)
    % This overlays points over cb_img and plots it.
    
    % Show image
    f = figure(); 
    a = axes(f);
    imshow(cb_img.get_gs(),[],'parent',a);
    hold(a,'on');
    
    % Plot points
    plot(points(:,1),points(:,2),'go','MarkerSize',6,'LineWidth',1);
    
    drawnow;
end