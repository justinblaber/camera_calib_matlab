function H_12 = homography_p2p_lin(p_1s, p_2s)
    % This will compute a homography directly using input points and linear
    % least squares fit.
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"
    %   p_2s - array; Nx2 array of points in perspective "2"
    %
    % Outputs:
    %   H_12 - array; 3x3 homography which transforms the points from
    %       perspective "1" to "2". Constraint of H_12(3, 3) = 1 is applied.

    % TODO: validate inputs. There must be at least four points?

    if size(p_1s, 1) ~= size(p_2s, 1)
        error('The same number of input points must be used to compute homography');
    end

    % Number of points
    num_points = size(p_1s, 1);

    % Perform normalization first
    T_1 = norm_mat(p_1s);
    p_1_augs = [p_1s ones(num_points, 1)]';
    p_1_norms = T_1*p_1_augs;

    T_2 = norm_mat(p_2s);
    p_2_augs = [p_2s ones(num_points, 1)]';
    p_2_norms = T_2*p_2_augs;

    % Compute homography with normalized points
    L = vertcat([p_1_norms' zeros(size(p_1_norms')) -p_2_norms(1, :)'.*p_1_norms'], ...
                [zeros(size(p_1_norms')) p_1_norms' -p_2_norms(2, :)'.*p_1_norms']);

    % Solution is the last column of V
    [~, ~, V] = svd(L);
    H_12_norm = reshape(V(:, end), 3, 3)';

    % "Undo" normalization to get desired homography
    H_12 = alg.safe_inv(T_2)*H_12_norm*T_1;

    % Normalize H_12(3, 3) to 1
    H_12 = H_12./H_12(3, 3);
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
