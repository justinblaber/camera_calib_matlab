function A = linear_intrinsic_params(homographies,cb_imgs)
    % This will compute the instrinsic parameters using a linear least 
    % squares fit given a set of homographies. This assumes a single alpha
    % and that the principle point is at the center of the image.
    %
    % Inputs:
    %   homographies - cell; cell of 3x3 homographies
    %   cb_imgs - class.img; calibration board image
    %
    % Outputs:
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %
    %       alpha - positive scalar;  f*k
    %       x_o - scalar; x component of image center
    %       y_o - scalar; y component of image center
              
    % Assumes alpha_x and alpha_y are equal. Also assumes x_o and y_o are
    % the center of the image.
    x_o = (cb_imgs(1).get_width()+1)/2;
    y_o = (cb_imgs(1).get_height()+1)/2;
    p_o_inv = [1 0 -x_o;
               0 1 -y_o;
               0 0  1];
    
    A = zeros(2*length(homographies),1);
    b = zeros(2*length(homographies),1);
    for i = 1:length(homographies)
        % Remove principle point from homography
        H_bar = p_o_inv*homographies{i};
        
        % Get orthogonal vanishing points
        v1 = H_bar(:,1);
        v2 = H_bar(:,2);
        v3 = (H_bar(:,1)+H_bar(:,2))./2;
        v4 = (H_bar(:,1)-H_bar(:,2))./2;
        
        % 
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
        
    % Solve for alpha
    alpha = sqrt(1/(pinv(A)*b));
    
    % Set output
    A = [alpha 0      x_o;
         0     alpha  y_o;
         0     0      1];
end