function points_2 = apply_homography(homography_1_2,points_1)
    % This will apply a homography to input points. Outputs are normalized
    % so the 3rd coordinate is 1 (and is omitted).
    %
    % Inputs:
    %   homography_1_2 - array; 3x3 homography
    %   points_1 - array; Nx2 array of points
    %
    % Outputs:
    %   points_2 - array; Nx2 array of points
    
    % Augment and transpose -> apply homography -> normalize by 3rd
    % coordinate -> untranspose
    points_2 = homography_1_2 * [points_1 ones(size(points_1,1),1)]';
    points_2 = (points_2(1:2,:)./points_2(3,:))';
end