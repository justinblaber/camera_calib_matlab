function homography_1_2 = homography(points_1,points_2,opts)
    % This will compute a homography using non-linear least squares fit 
    % which transforms the points in "perspective 1" to "perspective 2".
    %
    % Inputs:
    %   points_1 - array; Nx2 array of points
    %   points_2 - array; Nx2 array of points
    %   opts - struct; 
    %       .homography_it_cutoff - int; number of iterations performed for 
    %           nonlinear homography refinement
    %       .homography_norm_cutoff - scalar; cutoff for norm of difference
    %           of parameter vector for nonlinear homography refinement
    %
    % Outputs:
    %   homography_1_2 - array; 3x3 array which transforms the points from 
    %       1 to 2. Constraint of homography_1_2(3,3) = 1 is applied.
    
    % TODO: validate inputs. There must be at least four points? Any other
    % conditions?
        
    % Number of points
    num_points = size(points_1,1);
    
    % Get linear guess ---------------------------------------------------%    
    % Perform normalization first
    T_1 = norm_mat(points_1);
    points_1_aug = [points_1 ones(num_points,1)]';
    points_1_norm = T_1*points_1_aug;
    
    T_2 = norm_mat(points_2);
    points_2_aug = [points_2 ones(num_points,1)]';
    points_2_norm = T_2*points_2_aug;
    
    % Compute homography with normalized points
    L = vertcat([points_1_norm' zeros(size(points_1_norm')) -points_2_norm(1,:)'.*points_1_norm'], ...
                [zeros(size(points_1_norm')) points_1_norm' -points_2_norm(2,:)'.*points_1_norm']);    
    
    % Solution is the last column of V
    [~,~,V] = svd(L);    
    homography_norm_1_2 = reshape(V(:,end),3,3)';
    
    % "Undo" normalization to get desired homography
    homography_1_2 = T_2^-1*homography_norm_1_2*T_1;
    
    % Normalize homography_1_2(3,3) to 1
    homography_1_2 = homography_1_2./homography_1_2(3,3);
    
    % Perform nonlinear refinement ---------------------------------------%
    % Initialize homography parameter vector
    h = homography_1_2(1:8)';
     
    % Initialize jacobian and residual vector
    jacob = zeros(2*num_points,8);
    res = zeros(2*num_points,1);
    
    % Perform gauss newton iterations until convergence
    for it = 1:opts.homography_it_cutoff
        % Form homography from vector
        homography = ones(3,3);
        homography(1:8) = h;
        
        % Compute jacobian
        points_prime = homography*points_1_aug;
        u_prime = points_prime(1,:)';
        v_prime = points_prime(2,:)';
        w_prime = points_prime(3,:)';
        
        % x coords
        jacob(1:num_points,1) = points_1(:,1)./w_prime;
        jacob(1:num_points,3) = -u_prime.*points_1(:,1)./w_prime.^2;
        jacob(1:num_points,4) = points_1(:,2)./w_prime;
        jacob(1:num_points,6) = -u_prime.*points_1(:,2)./w_prime.^2;
        jacob(1:num_points,7) = 1./w_prime;
        
        % y coords
        jacob(num_points+1:2*num_points,2) = points_1(:,1)./w_prime;
        jacob(num_points+1:2*num_points,3) = -v_prime.*points_1(:,1)./w_prime.^2;
        jacob(num_points+1:2*num_points,5) = points_1(:,2)./w_prime;
        jacob(num_points+1:2*num_points,6) = -v_prime.*points_1(:,2)./w_prime.^2;
        jacob(num_points+1:2*num_points,8) = 1./w_prime;

        % Store residual
        res(1:num_points) = u_prime./w_prime - points_2(:,1);
        res(num_points+1:2*num_points,1) = v_prime./w_prime - points_2(:,2);

        % Get and store update
        delta_h = -lscov(jacob,res);
        h = h + delta_h;
        
        % Exit if change in distance is small
        if norm(delta_h) < opts.homography_norm_cutoff
            break
        end
    end    
    
    % Store final homography
    homography_1_2 = ones(3,3);
    homography_1_2(1:8) = h;
end

function T_norm = norm_mat(points)
    points_x = points(:,1);
    points_y = points(:,2);
    mean_x = mean(points_x);
    mean_y = mean(points_y);    
    sm = sqrt(2)*size(points,1)./sum(sqrt((points_x-mean_x).^2+(points_y-mean_y).^2));
    T_norm = [sm 0  -mean_x*sm;
              0  sm -mean_y*sm;
              0  0   1];
end
