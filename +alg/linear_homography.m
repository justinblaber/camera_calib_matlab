function homography_w_i = linear_homography(points_w, points_i)
    % This will compute a homography using linear least squares fit which 
    % transforms the points in "perspective w" to "perspective i".
    %
    % Inputs:
    %   points_w - array; Nx2 array of points
    %   points_i - array; Nx2 array of points
    %
    % Outputs:
    %   homography_w_i - array; 3x4 array which transforms the points from 
    %       w to i.
    
    % TODO: validate inputs. They must be the same size and there must be
    % least four non-collinear points (I think...)
        
    % Perform normalization first
    T_w = norm_mat(points_w);
    points_w_aug = [points_w ones(size(points_w,1),1)]';
    points_w_norm = T_w*points_w_aug;
    
    T_i = norm_mat(points_i);
    points_i_aug = [points_i ones(size(points_i,1),1)]';
    points_i_norm = T_i*points_i_aug;
    
    % Compute homography with normalized points
    L = vertcat([points_w_norm' zeros(size(points_w_norm')) -points_i_norm(1,:)'.*points_w_norm'], ...
                [zeros(size(points_w_norm')) points_w_norm' -points_i_norm(2,:)'.*points_w_norm']);    
    
    % Solution is the last column of V - note that contraint of norm(H) =
    % 1 is applied by svd.
    [~,~,V] = svd(L);    
    homography_norm_w_i = reshape(V(:,end),3,3)';
    
    % "Undo" normalization to get desired homography
    homography_w_i = T_i^-1*homography_norm_w_i*T_w;
end

function T_norm = norm_mat(points)
    points_x = points(:,1);
    points_y = points(:,2);
    mean_x = mean(points_x);
    mean_y = mean(points_y);    
    sm = sqrt(2)*size(points,1)./sum(sqrt((points_x-mean_x).^2 + (points_y-mean_y).^2));
    T_norm = [sm 0  -mean_x*sm;
              0  sm -mean_y*sm;
              0  0   1];
end