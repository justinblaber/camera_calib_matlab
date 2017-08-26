function plot_cb_points_disp(points1,points2,cb_img,a)
    % This overlays point displacements on cb_img and plots it.
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Show image
    imshow(cb_img.get_gs(),[],'parent',a);
    hold(a,'on');
    
    % Plot points
    plot(points1(:,1),points1(:,2),'bs','MarkerSize',6,'LineWidth',1,'parent',a);
    plot(points2(:,1),points2(:,2),'r+','MarkerSize',6,'LineWidth',1,'parent',a);
    
    % Plot displacements
    quiver(points1(:,1),points1(:,2),points2(:,1)-points1(:,1),points2(:,2)-points1(:,2),'color','g','LineWidth',1,'AutoScale','off','parent',a);
    
    drawnow;
end