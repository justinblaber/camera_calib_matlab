function points_h = apply_homography(homography,points)
    % This will apply homography to input points.
    %
    % Inputs:
    %   homography - array; 3x3 homography
    %   points - array; Nx2 array of points
    %
    % Outputs:
    %   points_h - array; Nx2 array of points
    
    % Augment and transpose -> Apply homography -> normalize by 3rd
    % coordinate -> untranspose
    points_h = homography * [points ones(size(points,1),1)]';
    points_h = (points_h(1:2,:)./points_h(3,:))';
end