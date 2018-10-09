function p_p_ds = p_p2p_p_d(p_ps,A,f_p_n2p_n_d,d)
    % Transforms points from pixels to distorted pixel coordinates.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of pixel points
    %   A - array; 3x3 camera matrix
    %   f_p_n2p_n_d - function handle; describes mapping between normalized
    %       coordinates and distorted normalized coordinates.
    %   d - array; Mx1 array of distortion coefficients
    % 
    % Outputs:
    %   p_p_ds - array; Nx2 array of distorted pixel points
    
    % Convert pixel coordinates to normalized coordinates
    p_ns = alg.p_p2p_n(p_ps,A);
    
    % Convert normalized coordinates to distorted normalized coordinates
    d_cell = num2cell(d);
    p_n_ds = f_p_n2p_n_d(p_ns(:,1),p_ns(:,2),d_cell{:});
    
    % Convert distorted normalized coordinates to distorted pixel
    % coordinates
    p_n_d_augs = [p_n_ds ones(size(p_ps,1),1)]';
    p_p_d_augs = A*p_n_d_augs;
    p_p_ds = [(p_p_d_augs(1,:)./p_p_d_augs(3,:))' (p_p_d_augs(2,:)./p_p_d_augs(3,:))'];
end