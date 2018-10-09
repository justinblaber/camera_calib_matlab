function p_ns = p_p2p_n(p_ps,A)
    % Transforms points from pixel coordinates to normalized coordinates.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of pixel points
    %   A - array; 3x3 camera matrix
    %   d - array; Mx1 array of distortion coefficients
    % 
    % Outputs:
    %   p_ns - array; Nx2 array of normalized points
    
    % Form augmented points in pixel coordinates
    p_p_augs = [p_ps ones(size(p_ps,1),1)]';
    
    % Get augmented normalized points
    p_n_augs = inv(A)*p_p_augs; %#ok<MINV>
    
    % Get normalized points
    p_ns = [(p_n_augs(1,:)./p_n_augs(3,:))' (p_n_augs(2,:)./p_n_augs(3,:))'];
end