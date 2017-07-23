function plot_cb_refine_points(points,cb_img,homography,cb_config,show_weights,a)
    % This will plot the points and the window around the points
        
    if ~exist('a','var')
        f = figure(); 
        a = axes(f);
    end
             
    % Plot points over calibration board image    
    cb_img.imshow(a);
    hold(a,'on');
    plot(points(:,1),points(:,2),'r+','parent',a);
    
    % Get half_window
    hw = half_window(homography,cb_config);
    
    % Get gaussian kernel - this is used to weight points closer to the
    % corner higher.
    gk = gauss_kernel(hw);    
    
    % Plot boxes around points   
    cb_gs = cb_img.get_gs();
    for i = 1:size(points,1)  
        % Get window around point; apply inverse homography to it in 
        % order to first bring it into world coordinates
        win_points_w = window_points(alg.apply_homography(homography^-1,points(i,:)), ...
                                     hw, ...
                                     cb_config); 

        % Apply homography to window points to bring them into image
        % coordinates
        win_points_i = alg.apply_homography(homography,win_points_w);
            
        % Make sure coords are withinbounds
        idx_inbounds = win_points_i(:,1) >= 1 & win_points_i(:,1) <= size(cb_gs,2) & ...
                       win_points_i(:,2) >= 1 & win_points_i(:,2) <= size(cb_gs,1);
        win_points_i = win_points_i(idx_inbounds,:);
        
        % Plot points
        if exist('show_weights','var') && show_weights
            % This is slow
            for j = 1:size(win_points_i,1)
                plot(win_points_i(j,1),win_points_i(j,2),'bo','Markersize',5*gk(j),'parent',a);
            end   
            drawnow
        else
            % Get four corner points
            win = 2*hw+1;
            p1 = [win_points_i(1,1) win_points_i(1,2)];
            p2 = [win_points_i(win,1) win_points_i(win,2)];
            p3 = [win_points_i(win*win,1) win_points_i(win*win,2)];
            p4 = [win_points_i(win*(win-1)+1,1) win_points_i(win*(win-1)+1,2)];
            plot([p1(1) p2(1) p3(1) p4(1) p1(1)],[p1(2) p2(2) p3(2) p4(2) p1(2)],'b','parent',a);
        end     
    end
        
    drawnow
end

function hw = half_window(homography,cb_config)
    % Get half window such that there is at least one sample per pixel
    % Get board points in world coordinates    
    board_points_w = alg.cb_points(cb_config);
    h = cb_config.num_rects_height;
    w = cb_config.num_rects_width;
        
    % Get outside points of four corner rectangles
    crp_w = vertcat(board_points_w(1,:), ...             % top-left
                    board_points_w(2,:), ...             % top-left-bottom
                    board_points_w(h,:), ...             % bottom-left-top
                    board_points_w(h+1,:), ...           % bottom-left
                    board_points_w(h+2,:), ...           % top-left-right                  
                    board_points_w((h+1)*2,:), ...       % bottom-left-right
                    board_points_w((w-1)*(h+1)+1,:), ... % top-right-left   
                    board_points_w(w*(h+1),:), ...       % bottom-right-left
                    board_points_w(w*(h+1)+1,:), ...     % top-right
                    board_points_w(w*(h+1)+2,:), ...     % top-right-bottom
                    board_points_w((w+1)*(h+1)-1,:), ... % bottom-right-top
                    board_points_w((w+1)*(h+1),:));      % bottom-right
                  
    % Apply homography
    crp_i = alg.apply_homography(homography,crp_w);
   
    % Get distances of outside sides of corner rectangles
    crd_i = vertcat(norm(crp_i(2,:) - crp_i(1,:)), ...  % tl - tlb
                    norm(crp_i(4,:) - crp_i(3,:)), ...  % blt - bl
                    norm(crp_i(5,:) - crp_i(1,:)), ...  % tlr - tl
                    norm(crp_i(6,:) - crp_i(4,:)), ...  % blr - bl
                    norm(crp_i(9,:) - crp_i(7,:)), ...  % tr - trl
                    norm(crp_i(12,:) - crp_i(8,:)), ... % br - brl
                    norm(crp_i(10,:) - crp_i(9,:)), ... % trb - tr
                    norm(crp_i(12,:) - crp_i(11,:)));   % br - brt
            
    % Set half-window based on largest length
    hw = ceil(max(crd_i*cb_config.refine_window_factor/2));
end

function gk = gauss_kernel(half_window)
    % Get gaussian kernel
    gk = fspecial('Gaussian', ...
                  [2*half_window+1 2*half_window+1], ...
                  half_window);
              
    % Scale so max intensity is 1
    gk = reshape(gk./max(gk(:)),[],1);
end

function win_points = window_points(point_w,half_window,cb_config)
    % Get grid of points in world coordinates
    [win_points_y, win_points_x] = ndgrid(linspace(point_w(2)-(cb_config.rect_height/2)*cb_config.refine_window_factor, ...
                                                   point_w(2)+(cb_config.rect_height/2)*cb_config.refine_window_factor, ...
                                                   2*half_window+1), ...
                                          linspace(point_w(1)-(cb_config.rect_width/2)*cb_config.refine_window_factor, ...
                                                   point_w(1)+(cb_config.rect_width/2)*cb_config.refine_window_factor, ...
                                                   2*half_window+1));        
    win_points = [win_points_x(:) win_points_y(:)];    
end