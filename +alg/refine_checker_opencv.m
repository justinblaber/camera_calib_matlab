function [p, cov_p] = refine_checker_opencv(array_dx, array_dy, W)
    % Performs "opencv" refinement of a checker center on input array
    % gradients.
    %
    % Inputs:
    %   array_dx - array; MxN array gradient in x direction
    %   array_dy - array; MxN array gradient in y direction
    %   W - array; optional MxN weight array
    %
    % Outputs:
    %   p - array; 1x2 refined checker center
    %   cov_p - array; 2x2 covariance of checker center

    if ~exist('W', 'var')
        W = ones(size(array_dx));
    end

    % Get coordinates of pixels
    bb_array = alg.bb_array(array_dx);
    [ys, xs] = alg.ndgrid_bb(bb_array);
    xs = xs(:);
    ys = ys(:);

    % Form linear system
    A = [array_dx(:) array_dy(:)];
    b = array_dx(:).*xs + array_dy(:).*ys;

    % Solve
    [p, ~, ~, cov_p] = alg.safe_lscov(A, b, W(:));
    p = p';
end
