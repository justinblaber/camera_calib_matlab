function H_12 = homography_c2e_nonlin(p_1s,p_2s,H_12_init,opts,cov)
    % This will compute a homography, assuming points in perspective "1"
    % are centers of circles while points in perspective "2" are centers
    % of ellipses, using non-linear least squares fit.
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"; centers of
    %       circles
    %   p_2s - array; Nx2 array of points in perspective "2"; centers of
    %       ellipses
    %   opts - struct; 
    %       .homography_c2e_it_cutoff - int; number of iterations performed
    %           for "c2e" nonlinear homography refinement
    %       .homography_c2e_norm_cutoff - scalar; cutoff for norm of 
    %           difference of parameter vector for nonlinear "c2e" 
    %           homography refinement
    %       .circle_radius - scalar; radius of circle in perspective "1"
    %           coordinates
    %   cov - array; optional 2*Nx2*N covariance array used for generalized
    %       least squares analysis
    %
    % Outputs:
    %   H_12 - array; 3x3 homography which transforms the points from
    %       perspective "1" to "2". Constraint of H_12(3,3) = 1 is applied.
    
    % TODO: validate inputs. There must be at least four points?
    
    if size(p_1s,1) ~= size(p_2s,1)
        error('The same number of input points must be used to compute homography');
    end
        
    % Number of points
    num_points = size(p_1s,1);
    
    % Get radius of circle in perspective "1" coordinates
    r_1 = opts.circle_radius;
    
    % Perform nonlinear refinement ---------------------------------------%
    % Initialize homography parameter vector; make sure H_12(3,3) is 1
    h = H_12_init(1:8)'./H_12_init(end);
    
    % Perform gauss newton iterations until convergence
    for it = 1:opts.homography_c2e_it_cutoff     
        % Form homography from vector
        H_12 = reshape([h; 1],3,3);
        
        % Compute jacobian
        jacob = alg.dp_dh_c2e(H_12,p_1s,r_1);
        jacob = jacob(:,1:end-1); % Remove last column since H_12(3,3) is constant
                
        % Get residual 
        res = reshape((alg.apply_homography_c2e(H_12,p_1s,r_1) - p_2s)',2*num_points,1);

        % Get and store update
        if ~exist('cov','var')
            delta_h = -lscov(jacob,res);
        else
            delta_h = -lscov(jacob,res,cov);
        end        
        h = h + delta_h;
        
        % Exit if change in distance is small
        if norm(delta_h) < opts.homography_c2e_norm_cutoff
            break
        end
    end  
    
    % Store final homography
    H_12 = reshape([h; 1],3,3);
end