function H_12 = homography_p2p_nonlin(p_1s,p_2s,H_12_init,opts,cov)
    % This will compute a homography directly using input points and 
    % non-linear least squares fit.
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"
    %   p_2s - array; Nx2 array of points in perspective "2"
    %   H_12_init - array; 3x3 initial guess of homography which transforms
    %       the points from perspective "1" to "2".
    %   opts - struct; 
    %       .homography_p2p_it_cutoff - int; number of iterations performed
    %           for "p2p" nonlinear homography refinement
    %       .homography_p2p_norm_cutoff - scalar; cutoff for norm of 
    %           difference of parameter vector for nonlinear "p2p" 
    %           homography refinement
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
    
    % Initialize homography parameter vector; make sure H_12(3,3) is 1
    params = H_12_init(1:8)'./H_12_init(end);
             
    % Perform gauss newton iterations until convergence
    for it = 1:opts.homography_p2p_it_cutoff
        % Form homography from vector
        H_12 = reshape([params; 1],3,3);
        
        % Compute jacobian
        jacob = alg.dp_dh_p2p(p_1s,H_12);
        jacob = jacob(:,1:end-1); % Remove last column since H_12(3,3) is constant
                     
        % Compute residual
        res = reshape((alg.apply_homography_p2p(p_1s,H_12) - p_2s)',2*num_points,1);

        % Get and store update
        if ~exist('cov','var')
            delta_params = -lscov(jacob,res);
        else
            delta_params = -lscov(jacob,res,cov);
        end        
        params = params + delta_params;
        
        % Exit if change in distance is small
        if norm(delta_params) < opts.homography_p2p_norm_cutoff
            break
        end
    end    
    
    % Store final homography
    H_12 = reshape([params; 1],3,3);
end
