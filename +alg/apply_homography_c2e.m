function p_2s = apply_homography_c2e(p_1s,H_12,r_1)
    % This will apply a homography assuming points in perspective "1"
    % are centers of circles while points in perspective "2" are centers
    % of ellipses. Outputs are normalized so the 3rd coordinate is 1 
    % (and is omitted).
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"; centers of
    %       circles
    %   H_12 - array; 3x3 homography which transforms the points from
    %       perspective "1" to "2".
    %   r_1 - scalar; radius of circle in perspective "1"
    %
    % Outputs:
    %   p_2s - array; Nx2 array of points in perspective "2"; centers of
    %       ellipses
    
    % TODO: Find matrix equation equivalent of this    
    u_prime = -H_12(3,1)*((p_1s(:,1)*H_12(1,3))/r_1^2 + (H_12(1,1)*(p_1s(:,1).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(1,2).*p_1s(:,2))/r_1^2) - H_12(3,2)*((H_12(1,3)*p_1s(:,2))/r_1^2 + (H_12(1,2)*(p_1s(:,2).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(1,1).*p_1s(:,2))/r_1^2) - H_12(3,3)*(H_12(1,3)/r_1^2 + (p_1s(:,1)*H_12(1,1))/r_1^2 + (H_12(1,2)*p_1s(:,2))/r_1^2);
    v_prime = -H_12(3,1)*((p_1s(:,1)*H_12(2,3))/r_1^2 + (H_12(2,1)*(p_1s(:,1).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(2,2).*p_1s(:,2))/r_1^2) - H_12(3,2)*((H_12(2,3)*p_1s(:,2))/r_1^2 + (H_12(2,2)*(p_1s(:,2).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(2,1).*p_1s(:,2))/r_1^2) - H_12(3,3)*(H_12(2,3)/r_1^2 + (p_1s(:,1)*H_12(2,1))/r_1^2 + (H_12(2,2)*p_1s(:,2))/r_1^2);
    w_prime = -H_12(3,1)*((p_1s(:,1)*H_12(3,3))/r_1^2 + (H_12(3,1)*(p_1s(:,1).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(3,2).*p_1s(:,2))/r_1^2) - H_12(3,2)*((H_12(3,3)*p_1s(:,2))/r_1^2 + (H_12(3,2)*(p_1s(:,2).^2 - r_1^2))/r_1^2 + (p_1s(:,1)*H_12(3,1).*p_1s(:,2))/r_1^2) - H_12(3,3)*(H_12(3,3)/r_1^2 + (p_1s(:,1)*H_12(3,1))/r_1^2 + (H_12(3,2)*p_1s(:,2))/r_1^2);
    
    p_2s = [u_prime./w_prime v_prime./w_prime];
end
