function e = refine_ellipse_dualconic(array_dx, array_dy, W)
    % Performs "dual conic" refinement of an ellipse
    %
    % Inputs:
    %   array_dx - array; MxN array gradient in x direction
    %   array_dy - array; MxN array gradient in y direction
    %   W - array; optional MxN weight array
    %
    % Outputs:
    %   e - array; 5x1 ellipse matrix stored as:
    %       e(1) = h; x component of center of ellipse
    %       e(2) = k; y component of center of ellipse
    %       e(3) = a; major axis length
    %       e(4) = b; minor axis length
    %       e(5) = alpha; rotation of major axis

    if ~exist('W', 'var')
        W = ones(size(array_dx));
    end

    % Fit conic
    Aq = alg.fit_conic(array_dx, array_dy, W);

    % TODO: get covariance of ellipse parameters

    % Convert conic to ellipse
    e = alg.conic2ellipse(Aq);
end
