function jacob = dp_dh_p2p(H_12,p_1s)
    % This will compute the jacobian of the points in perspective "2" with 
    % respect to the input homography and evaluated at points in 
    % perspective "1" directly.
    %
    % Inputs:
    %   H_12 - array; 3x3 homography which transforms the points from
    %       perspective "1" to "2".
    %   p_1s - array; Nx2 array of points in perspective "1"
    %
    % Outputs:
    %   jacob - array; 2*Nx9 array. 
    %       Format of jacobian is:
    %
    %               dh11 dh21 dh31 dh12 dh22 dh32 dh13 dh23 dh33
    %       dx_2_1
    %       dy_2_1
    %          .
    %          .
    %          .
    %       dx_2_N
    %       dy_2_N
               
    % Get number of points
    num_points = size(p_1s,1);    
    
    % Compute jacobian
    p_prime = H_12*[p_1s ones(num_points,1)]';
    u_prime = p_prime(1,:);
    v_prime = p_prime(2,:);
    w_prime = p_prime(3,:);

    jacob = [p_1s(:,1)'./w_prime;
             zeros(1,num_points);
             -u_prime.*p_1s(:,1)'./w_prime.^2;
             p_1s(:,2)'./w_prime;
             zeros(1,num_points);
             -u_prime.*p_1s(:,2)'./w_prime.^2;
             1./w_prime;
             zeros(1,num_points);
             -u_prime./w_prime.^2;
             zeros(1,num_points);
             p_1s(:,1)'./w_prime;
             -v_prime.*p_1s(:,1)'./w_prime.^2;
             zeros(1,num_points);
             p_1s(:,2)'./w_prime;
             -v_prime.*p_1s(:,2)'./w_prime.^2;
             zeros(1,num_points);
             1./w_prime;
             -v_prime./w_prime.^2];        
    jacob = reshape(jacob,9,2*num_points)';    
end