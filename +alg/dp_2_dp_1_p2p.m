function jacob = dp_2_dp_1_p2p(p_1s, H_12)
    % This will compute the jacobian of the points in perspective "2" with
    % respect to points in perspective "1".
    %
    % Inputs:
    %   p_1s - array; Nx2 array of points in perspective "1"
    %   H_12 - array; 3x3 homography which transforms the points from
    %       perspective "1" to "2".
    %
    % Outputs:
    %   jacob - array; 2*Nx2*N array.
    %       Format of jacobian is:
    %
    %               dx_1_1 dy_1_1 ... dx_1_N dy_1_N
    %       dx_2_1
    %       dy_2_1
    %          .
    %          .
    %          .
    %       dx_2_N
    %       dy_2_N

    % Get number of points
    num_points = size(p_1s, 1);

    % Compute jacobian
    p_prime = H_12*[p_1s ones(num_points, 1)]';
    u_prime = p_prime(1, :)';
    v_prime = p_prime(2, :)';
    w_prime = p_prime(3, :)';

    dx_2_dx_1 = H_12(1, 1)./w_prime - (H_12(3, 1).*u_prime)./w_prime.^2;
    dx_2_dy_1 = H_12(1, 2)./w_prime - (H_12(3, 2).*u_prime)./w_prime.^2;
    dy_2_dx_1 = H_12(2, 1)./w_prime - (H_12(3, 1).*v_prime)./w_prime.^2;
    dy_2_dy_1 = H_12(2, 2)./w_prime - (H_12(3, 2).*v_prime)./w_prime.^2;

    dp_2_dx_1 = reshape([dx_2_dx_1 dy_2_dx_1]', [], 1);
    dp_2_dy_1 = reshape([dx_2_dy_1 dy_2_dy_1]', [], 1);

    % TODO: find a better way to construct sparse diagonal jacobian,
    % as this is probably a bottleneck

    jacob_cell = mat2cell(sparse([dp_2_dx_1 dp_2_dy_1]), 2*ones(1, size(p_1s, 1)), 2);
    jacob = blkdiag(jacob_cell{:});
end
