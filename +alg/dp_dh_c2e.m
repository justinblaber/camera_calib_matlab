function jacob = dp_dh_c2e(p_1s,H_12,r_1)
    % This will compute the jacobian of the points in perspective "2", 
    % which are centers of ellipses, with respect to the input homography
    % and evaluated at points in perspective "1", which are centers of
    % circles.
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"; centers of
    %       circles
    %   H_12 - array; 3x3 homography which transforms the points from
    %       perspective "1" to "2".
    %   r_1 - scalar; radius of circle in perspective "1"
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
    % TODO: Find matrix equation equivalent of this    
    u_prime = -H_12(3,1)*((p_1s(:,1)*H_12(1,3))/r_1^2 + (H_12(1,1)*(p_1s(:,1).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(1,2).*p_1s(:,2))/r_1^2) - H_12(3,2)*((H_12(1,3)*p_1s(:,2))/r_1^2 + (H_12(1,2)*(p_1s(:,2).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(1,1).*p_1s(:,2))/r_1^2) - H_12(3,3)*(H_12(1,3)/r_1^2 + (p_1s(:,1)*H_12(1,1))/r_1^2 + (H_12(1,2)*p_1s(:,2))/r_1^2);
    v_prime = -H_12(3,1)*((p_1s(:,1)*H_12(2,3))/r_1^2 + (H_12(2,1)*(p_1s(:,1).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(2,2).*p_1s(:,2))/r_1^2) - H_12(3,2)*((H_12(2,3)*p_1s(:,2))/r_1^2 + (H_12(2,2)*(p_1s(:,2).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(2,1).*p_1s(:,2))/r_1^2) - H_12(3,3)*(H_12(2,3)/r_1^2 + (p_1s(:,1)*H_12(2,1))/r_1^2 + (H_12(2,2)*p_1s(:,2))/r_1^2);
    w_prime = -H_12(3,1)*((p_1s(:,1)*H_12(3,3))/r_1^2 + (H_12(3,1)*(p_1s(:,1).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(3,2).*p_1s(:,2))/r_1^2) - H_12(3,2)*((H_12(3,3)*p_1s(:,2))/r_1^2 + (H_12(3,2)*(p_1s(:,2).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(3,1).*p_1s(:,2))/r_1^2) - H_12(3,3)*(H_12(3,3)/r_1^2 + (p_1s(:,1)*H_12(3,1))/r_1^2 + (H_12(3,2)*p_1s(:,2))/r_1^2);
    
    jacob = [(-(p_1s(:,1)*H_12(3,3) + p_1s(:,1).^2*H_12(3,1) - H_12(3,1)*r_1^2 + p_1s(:,1)*H_12(3,2).*p_1s(:,2))./(r_1^2*w_prime))';
             zeros(1,num_points);
             ((2*u_prime.*(p_1s(:,1)*H_12(3,3) + p_1s(:,1).^2*H_12(3,1) - H_12(3,1)*r_1^2 + p_1s(:,1)*H_12(3,2).*p_1s(:,2)))./(r_1^2.*w_prime.^2) - (p_1s(:,1)*H_12(1,3) + p_1s(:,1).^2*H_12(1,1) - H_12(1,1)*r_1^2 + p_1s(:,1)*H_12(1,2).*p_1s(:,2))./(r_1^2*w_prime))';
             (-(H_12(3,3)*p_1s(:,2) + H_12(3,2)*p_1s(:,2).^2 - H_12(3,2)*r_1^2 + p_1s(:,1)*H_12(3,1).*p_1s(:,2))./(r_1^2*w_prime))';
             zeros(1,num_points);
             ((2*u_prime.*(H_12(3,3)*p_1s(:,2) + H_12(3,2)*p_1s(:,2).^2 - H_12(3,2)*r_1^2 + p_1s(:,1)*H_12(3,1).*p_1s(:,2)))./(r_1^2*w_prime.^2) - (H_12(1,3)*p_1s(:,2) + H_12(1,2)*p_1s(:,2).^2 - H_12(1,2)*r_1^2 + p_1s(:,1)*H_12(1,1).*p_1s(:,2))./(r_1^2*w_prime))';
             (-(H_12(3,3) + p_1s(:,1)*H_12(3,1) + H_12(3,2)*p_1s(:,2))./(r_1^2*w_prime))';
             zeros(1,num_points);
             ((2*u_prime.*(H_12(3,3) + p_1s(:,1)*H_12(3,1) + H_12(3,2)*p_1s(:,2)))./(r_1^2*w_prime.^2) - (H_12(1,3) + p_1s(:,1)*H_12(1,1) + H_12(1,2)*p_1s(:,2))./(r_1^2*w_prime))';
             zeros(1,num_points);
             (-(p_1s(:,1)*H_12(3,3) + p_1s(:,1).^2*H_12(3,1) - H_12(3,1)*r_1^2 + p_1s(:,1)*H_12(3,2).*p_1s(:,2))./(r_1^2*w_prime))';
             ((2*v_prime.*(p_1s(:,1)*H_12(3,3) + p_1s(:,1).^2*H_12(3,1) - H_12(3,1)*r_1^2 + p_1s(:,1)*H_12(3,2).*p_1s(:,2)))./(r_1^2*w_prime.^2) - (p_1s(:,1)*H_12(2,3) + p_1s(:,1).^2*H_12(2,1) - H_12(2,1)*r_1^2 + p_1s(:,1)*H_12(2,2).*p_1s(:,2))./(r_1^2*w_prime))';
             zeros(1,num_points);
             (-(H_12(3,3)*p_1s(:,2) + H_12(3,2)*p_1s(:,2).^2 - H_12(3,2)*r_1^2 + p_1s(:,1)*H_12(3,1).*p_1s(:,2))./(r_1^2*w_prime))';
             ((2*v_prime.*(H_12(3,3)*p_1s(:,2) + H_12(3,2)*p_1s(:,2).^2 - H_12(3,2)*r_1^2 + p_1s(:,1)*H_12(3,1).*p_1s(:,2)))./(r_1^2*w_prime.^2) - (H_12(2,3)*p_1s(:,2) + H_12(2,2)*p_1s(:,2).^2 - H_12(2,2)*r_1^2 + p_1s(:,1)*H_12(2,1).*p_1s(:,2))./(r_1^2*w_prime))';
             zeros(1,num_points);
             (-(H_12(3,3) + p_1s(:,1)*H_12(3,1) + H_12(3,2)*p_1s(:,2))./(r_1^2*w_prime))';
             ((2*v_prime.*(H_12(3,3) + p_1s(:,1)*H_12(3,1) + H_12(3,2)*p_1s(:,2)))./(r_1^2*w_prime.^2) - (H_12(2,3) + p_1s(:,1)*H_12(2,1) + H_12(2,2)*p_1s(:,2))./(r_1^2*w_prime))'];
    jacob = reshape(jacob,9,2*num_points)';
end
