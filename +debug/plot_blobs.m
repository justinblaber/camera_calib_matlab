function plot_blobs(blobs,array,calib_config,a)
    % Plots blobs

    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
    cla(a);
    
    % Plot background image
    imshow(array,[],'parent',a);
    hold(a,'on');
    
    % Get k
    k = 2^(1/calib_config.blob_detect_s);
    
    % Plot blobs    
    for i = 1:length(blobs)
        % Get eigenvalue decomposition of M
        [V,D] = eig(blobs(i).M);
        
        % Ellipse sides are proportional to sqrt of eigenvalues
        r_pix = sqrt(2)*(1+(k-1)/2)*blobs(i).sigma;
        scale_factor = 2*r_pix/(sqrt(D(1,1))+sqrt(D(2,2)));
        r1_pix = sqrt(D(1,1))*scale_factor;
        r2_pix = sqrt(D(2,2))*scale_factor;
        
        % Rotation is based on first axis
        rot = -atan2(V(1,1),V(2,1));
        
        % ellipse does not have an input argument for axes unfortunately
        util.ellipse(r1_pix, ...
                     r2_pix, ...
                     rot, ...
                     blobs(i).x, ...
                     blobs(i).y, ...
                     'r');
    end
    
    % Remove hold
    drawnow
    hold(a,'off');
end