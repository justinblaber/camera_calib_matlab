function debug_cb_points(points,cb_img,a)
    % This overlays points over cb_img and plots it.
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    
    % Show image
    cb_img.imshow(a);
    hold(a,'on');
    
    % Plot points
    plot(points(:,1),points(:,2),'go','MarkerSize',6,'LineWidth',1,'parent',a);
    
    drawnow;
end