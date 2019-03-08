function Aq = fit_conic(array_dx, array_dy, W)
    % Given input array gradients, this will attempt to find the parameters
    % of a conic matrix.
    %
    % Inputs:
    %   array_dx - array; MxN array gradient in x direction
    %   array_dy - array; MxN array gradient in y direction
    %   W - array; optional MxN weight array
    %
    % Outputs:
    %   Aq - array; 3x3 conic matrix stored as:
    %       [A   B/2 D/2;
    %        B/2 C   E/2;
    %        D/2 E/2 F];

    if ~exist('W', 'var')
        W = ones(size(array_dx));
    end

    % Get coordinates of pixels
    bb_array = alg.bb_array(array_dx);
    [ys, xs] = alg.ndgrid_bb(bb_array);
    ps = [xs(:) ys(:)];

    % Normalize coordinates; shift the origin of the coordinate system
    % to the center of the image, and then scale the axes such that the
    % mean distance to the center becomes sqrt(2); this should be
    % "good enough".
    T = norm_mat(ps);
    p_augs = [ps ones(size(ps, 1), 1)]';
    p_norms = T*p_augs;

    % Get homogeneous coordinates of lines
    ls = [array_dx(:) array_dy(:) -(array_dx(:).*p_norms(1, :)' + array_dy(:).*p_norms(2, :)')];

    % Form linear equations and solve for inverse conic
    A = [ls(:, 1).^2 ls(:, 1).*ls(:, 2) ls(:, 2).^2 ls(:, 1).*ls(:, 3) ls(:, 2).*ls(:, 3)];
    b = -ls(:, 3).^2;

    % Solve
    aq = alg.safe_lscov(A, b, W(:));

    % Get conic matrix
    Aq_inv = [aq(1)   aq(2)/2 aq(4)/2;
              aq(2)/2 aq(3)   aq(5)/2;
              aq(4)/2 aq(5)/2 1];
    Aq = alg.safe_inv(Aq_inv);

    % Rescale conic matrix to take normalization into account
    Aq = T'*Aq*T;
end

function T_norm = norm_mat(ps)
    xs = ps(:, 1);
    ys = ps(:, 2);
    mean_x = mean(xs);
    mean_y = mean(ys);
    s_m = sqrt(2)*size(ps, 1)./sum(sqrt((xs-mean_x).^2+(ys-mean_y).^2));
    T_norm = [s_m 0   -mean_x*s_m;
              0   s_m -mean_y*s_m;
              0   0    1];
end
