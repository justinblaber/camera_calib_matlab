function A = linear_intrinsic_params(homographies)
    % This will compute the instrinsic parameters using a linear least 
    % squares fit given a set of homographies.
    %
    % Inputs:
    %   homographies - cell; cell of 3x3 homographies
    %
    % Outputs:
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]
    %
    %       alpha_x - positive scalar;  f*kx
    %       alpha_y - positive scalar;  f*ky
    %       x_o - scalar; x component of image center
    %       y_o - scalar; y component of image center
          
    % TODO: validate 2 or more distinct homographies have been used which
    % is required to compute A with 4 parameters.
    
    % W is V in my notes, but V is also used in SVD notation, so use W here
    W = zeros(2*length(homographies),5);
    for i = 1:length(homographies)
        W(2*i-1,:) = w(homographies{i},1,2);
        W(2*i,:) = w(homographies{i},1,1) - w(homographies{i},2,2);
    end
        
    % Solution is the last column of V
    [~,~,V] = svd(W);    
    b = V(:,end);
    
    % Form B
    B = zeros(3,3);
    B(1,1) = b(1);
    B(2,2) = b(2);
    B(1,3) = b(3);
    B(2,3) = b(4);
    B(3,3) = b(5);
    % Symmetric
    B(3,1) = B(1,3);
    B(3,2) = B(2,3);
    
    % Get intrinsic parameters     
    alpha_x = sqrt(B(3,3)/B(1,1)-B(1,3)^2/B(1,1)^2-B(2,3)^2/(B(2,2)*B(1,1))); % Use positive value
    alpha_y = sqrt(B(3,3)/B(2,2)-B(1,3)^2/(B(1,1)*B(2,2))-B(2,3)^2/B(2,2)^2); % Use positive value   
    x_o = -B(1,3)/B(1,1);
    y_o = -B(2,3)/B(2,2);
    
    % Set output
    A = [alpha_x    0       x_o;
         0          alpha_y y_o;
         0          0       1];
end

function w_ij = w(h,i,j)
    w_ij = [h(1,j)*h(1,i) h(2,j)*h(2,i) h(3,j)*h(1,i)+h(1,j)*h(3,i) h(3,j)*h(2,i)+h(2,j)*h(3,i) h(3,j)*h(3,i)];
end