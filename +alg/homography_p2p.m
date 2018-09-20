function H_12 = homography_p2p(p_1s,p_2s,opts,cov)
    % This will compute a homography directly using input points and 
    % non-linear least squares fit.
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"
    %   p_2s - array; Nx2 array of points in perspective "2"
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
    
    % Compute initial guess with linear least squares
    H_12_init = alg.homography_p2p_lin(p_1s,p_2s);
    
    % Compute nonlinear optimized homography
    if ~exist('cov','var')
        H_12 = alg.homography_p2p_nonlin(p_1s,p_2s,H_12_init,opts);
    else
        H_12 = alg.homography_p2p_nonlin(p_1s,p_2s,H_12_init,opts,cov);
    end
end