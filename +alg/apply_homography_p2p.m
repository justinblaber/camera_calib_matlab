function p_2s = apply_homography_p2p(p_1s,H_12)
    % This will apply a homography directly to input points. Outputs are
    % normalized so the 3rd coordinate is 1 (and is omitted).
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"
    %   H_12 - array; 3x3 homography
    %
    % Outputs:
    %   p_2s - array; Nx2 array of points in perspective "2"
    
    % Augment and transpose -> apply homography -> normalize by 3rd
    % coordinate -> untranspose
    p_2s = H_12 * [p_1s ones(size(p_1s,1),1)]';
    p_2s = (p_2s(1:2,:)./p_2s(3,:))';
end
