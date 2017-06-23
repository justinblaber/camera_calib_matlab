function points = apply_inv_homography(homography,points_h)
    % This will apply the inverse homography to input points.
    %
    % Inputs:
    %   homography - array; 3x3 homography
    %   points_h - array; Nx2 array of points
    %
    % Outputs:
    %   points - array; Nx2 array of points
    
    % TODO: Add check to make sure homography isn't singular
    
    % Augment and transpose -> Apply inverse homography -> normalize by 3rd
    % coordinate -> untranspose
    points = homography^-1 * [points_h ones(size(points_h,1),1)]';
    points = (points(1:2,:)./points(3,:))';
end