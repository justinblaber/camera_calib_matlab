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
                'MarkerFaceColor',colors(idx,:),'MarkerFaceAlpha',alphas(idx), ...
                'MarkerEdgeAlpha',0,'parent',a);   
    end
    
    % Get infinity norm of res for plot
    max_res = max(cellfun(@(x)max(abs(x(:))),res));
    
    % Plot dashed line to indicate zero
    plot([0 0],[-max_res max_res],'--r','parent',a);
    plot([-max_res max_res],[0 0],'--r','parent',a);
        
    % Format plot
    set(a,'xlim',[-max_res max_res],'ylim',[-max_res max_res]);
    daspect(a,[1 1 1]);
    
    % Remove hold
    hold(a,'off');
end