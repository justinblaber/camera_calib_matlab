function corners = refine_corners(corners,cb_img,rw)
    % This will return subpixel corner coordinates given an initial guess,
    % calibration board image, and refinement window. This uses the
    % technique of getting the dot product along a window between the
    % displacement vector and gradient.
    %
    % Inputs:
    %   corners_init - array; nx2 array of points
    %   cb_img - class.img; calibration board image
    %   rw - scalar; 2*rw+1 is the size of the window around the point 
    %       which is used to gather points to form a constrained set of 
    %       equations.
    %
    % Outputs:
    %   corners - array; nx2 array of sub pixel points.
        
    % Get calibration board image
    cb_gs = cb_img.get_gs();
        
    % Get gradient images
    cb_gs_dx = imfilter(cb_gs,-fspecial('sobel')');
    cb_gs_dy = imfilter(cb_gs,-fspecial('sobel'));
                   
    % Perform iterations until convergence
    it_cutoff = 10;
    norm_cutoff = 0.05;
    for i = 1:size(corners,1)     
        for it = 1:it_cutoff
            corner_prev = corners(i,:);
            corners(i,:) = refine_corners_it(corner_prev,cb_gs_dx,cb_gs_dy,rw);  
            if norm(corner_prev-corners(i,:)) < norm_cutoff
                break
            end
        end  
        disp(['Iterations: ' num2str(it)]);
        disp(['Norm: ' num2str(norm(corner_prev-corners(i,:)))]);
    end    
end

function corners = refine_corners_it(corners,cb_gs_dx,cb_gs_dy,rw)
    % Get window around initial corner
    width = 2*rw+1;
    x_w = [repmat(corners(1,1)-rw,1,width) corners(1,1)-rw:corners(1,1)+rw repmat(corners(1,1)+rw,1,width) corners(1,1)+rw:-1:corners(1,1)-rw]';
    y_w = [corners(1,2)-rw:corners(1,2)+rw repmat(corners(1,2)+rw,1,width) corners(1,2)+rw:-1:corners(1,2)-rw repmat(corners(1,2)-rw,1,width)]';

    % Make sure coords are withinbounds
    idx_inbounds = x_w >= 1 & x_w <= size(cb_gs_dx,2) & y_w >= 1 & y_w <= size(cb_gs_dx,1);
    x_w = x_w(idx_inbounds);
    y_w = y_w(idx_inbounds);

    cb_gs_dx_window = alg.array_interp(cb_gs_dx,[x_w y_w]);
    cb_gs_dy_window = alg.array_interp(cb_gs_dy,[x_w y_w]);

    A = [cb_gs_dx_window cb_gs_dy_window];
    b = (cb_gs_dx_window.*x_w + cb_gs_dy_window.*y_w);

    corners(1,:) = (pinv(A)*b)';  
end