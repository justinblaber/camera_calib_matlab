function plot_res(res,colors,alphas,a)
    % This plots residuals
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Show image
    hold(a,'on');
    
    % Plot points; plot highest alpha last
    [~,idx_sorted] = sort(alphas);
    for i = 1:length(res)
        idx = idx_sorted(i);
        scatter(res{idx}(:,1),res{idx}(:,2),12, ...
                'MarkerFaceColor',colors(idx,:), 'MarkerFaceAlpha',alphas(idx), ...
                'MarkerEdgeAlpha',0,'parent',a);        
    end
    
    % Plot dashed line
    plot([0 0],[-0.5 0.5],'--r','parent',a);
    plot([-0.5 0.5],[0 0],'--r','parent',a);
    
    % Format plot
    set(a,'xlim',[-0.5 0.5],'ylim',[-0.5 0.5]);
    daspect(a,[1 1 1]);
    
    % Remove hold
    drawnow
    hold(a,'off');
end