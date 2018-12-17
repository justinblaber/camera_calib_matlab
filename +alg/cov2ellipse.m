function e = cov2ellipse(cov, p)
    % Converts input covariance matrix and point to an ellipse
    %
    % Inputs:
    %   cov - array; 2x2 covariance array
    %   p - array; 1x2 point
    %
    % Outputs:
    %   e - array; 5x1 ellipse matrix stored as:
    %       e(1) = h; x component of center of ellipse
    %       e(2) = k; y component of center of ellipse
    %       e(3) = a; major axis length
    %       e(4) = b; minor axis length
    %       e(5) = alpha; rotation of major axis

    if any(~isfinite(cov(:)))
        e = nan(5, 1);
        return
    end

    % Calculate eigenvectors and eigenvalues
    [V, D] = eig(cov);
    [D, idx_sorted] = sort(diag(D));
    V = V(:, idx_sorted); % Largest eigenvector (major axis) is second column

    % Major and minor axes length are sqrt of eigenvalues
    r1 = sqrt(D(2)); % Major
    r2 = sqrt(D(1)); % Minor

    % Get rotation of major axis
    rot = atan2(V(2, 2), V(1, 2)); % atan2(Y, X) of largest eigenvector

    % Store ellipse parameters
    e = [p(1) p(2) r1 r2 rot]';
end
