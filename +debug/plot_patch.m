function plot_patch(patch,template,num,cc_val,a)
    % This plots detected patch and overlays the template
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Show patch
    imshow(patch,[],'Parent',a);
    title(a,[num2str(num) ' (CC val: ' num2str(cc_val) ')'],'FontSize',7);  

    % Plot boundaries of template over patch
    hold(a,'on');      
    B = bwboundaries(template < 0.5);
    for k = 1:numel(B)
        boundary = B{k};
        plot(a, boundary(:,2),boundary(:,1),'Color','g','LineWidth',1);
    end
    
    % Remove hold
    hold(a,'off');
end
