function A = init_intrinsic_params(homographies,width,height)
    % This will initialize the intrinsic parameters of a given set of 
    % homographies. This assumes a single alpha and that the principle
    % point is at the center of the image. 
    %
    % This was taken from Bouguet's calibration toolbox. It gives pretty
    % good results from the tests I've done.
    %
    % Inputs:
    %   homographies - cell; cell of 3x3 homographies
    %   width - scalar; width of the calibration board image
    %   height - scalar; height of the calibration board image
    %
    % Outputs:
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %
    %       alpha - positive scalar; alpha = f*kx = f*ky
    %       x_o - scalar; x component of image center
    %       y_o - scalar; y component of image center
    
    % Set principle point
    x_o = (width+1)/2;
    y_o = (height+1)/2;
    
    % p_o_inv removes the principle point from the homography so alpha can
    % be computed.
    p_o_inv = [1 0 -x_o;
               0 1 -y_o;
               0 0  1];
           
    % Fill A and b
    A = zeros(2*length(homographies),1);
    b = zeros(2*length(homographies),1);
    for i = 1:length(homographies)
        % Remove principle point from homography
        H_bar = p_o_inv*homographies{i};
        
        % Get orthogonal vanishing points
        v1 = H_bar(:,1);
        v2 = H_bar(:,2);
        v3 = H_bar(:,1)+H_bar(:,2);
        v4 = H_bar(:,1)-H_bar(:,2);
        
        % Normalize vanishing points
        v1 = v1./norm(v1);
        v2 = v2./norm(v2);
        v3 = v3./norm(v3);
        v4 = v4./norm(v4);
        
        % Form constraints
        A(2*(i-1)+1:2*i) = [v1(1)*v2(1) + v1(2)*v2(2);
                            v3(1)*v4(1) + v3(2)*v4(2)];
        b(2*(i-1)+1:2*i) = [-v1(3)*v2(3);
                            -v3(3)*v4(3)];
    end
        
    % Get x by finding value which makes projection of A on b equal to b.
    % I believe this tends to prevent over estimates when A and b are not
    % collinear.
    alpha_squared = dot(b,A)/dot(b,b);
    if alpha_squared < 0
        error(['Alpha squared is equal to: ' num2str(alpha_squared) '. ' ...
               'This quantity must be positive. Are calibration board ' ...
               'parameters correct?']);
    end    
    alpha = sqrt(alpha_squared);
    
    % Set A
    A = [alpha 0      x_o;
         0     alpha  y_o;
         0     0      1];
end