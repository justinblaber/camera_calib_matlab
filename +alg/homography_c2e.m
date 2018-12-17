function H_12 = homography_c2e(p_1s, p_2s, opts, cov)
    % This will compute a homography, assuming points in perspecitive "1"
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
    %       perspective "1" to "2". Constraint of H_12(3, 3) = 1 is applied.

    % Compute initial guess with "p2p" linear least squares; should suffice
    % for an initial guess.
    H_12_init = alg.homography_p2p_lin(p_1s, p_2s);

    % Compute nonlinear optimized homography
    if ~exist('cov', 'var')
        H_12 = alg.homography_c2e_nonlin(p_1s, p_2s, H_12_init, opts);
    else
        H_12 = alg.homography_c2e_nonlin(p_1s, p_2s, H_12_init, opts, cov);
    end
end
