function jacob = dp_p_d_dp_p(p_ps, f_dp_p_d_dx_p, f_dp_p_d_dy_p, a, d)
    % This will compute the jacobian of distorted pixel points wrt pixel
    % points.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of pixel points
    %   f_dp_p_d_dx_p - function handle; derivative of p_p2p_p_d wrt x_p
    %   f_dp_p_d_dy_p - function handle; derivative of p_p2p_p_d wrt y_p
    %   a - array; 3x1 array containing:
    %       [alpha; x_o; y_o]
    %   d - array; Mx1 array of distortion coefficients
    %
    % Outputs:
    %   jacob - sparse array; 2*Nx2*N array.
    %       Format of jacobian is:
    %
    %                dx_p_1 dy_p_1 ... dx_p_N dy_p_N
    %       dx_p_d_1
    %       dy_p_d_1
    %          .
    %          .
    %          .
    %       dx_p_d_N
    %       dy_p_d_N

    % Compute partial derivatives
    d_cell = num2cell(d);
    dp_p_d_dx_p = f_dp_p_d_dx_p(p_ps(:, 1), ...
                                p_ps(:, 2), ...
                                a(1), ...
                                a(2), ...
                                a(3), ...
                                d_cell{:});
    dp_p_d_dy_p = f_dp_p_d_dy_p(p_ps(:, 1), ...
                                p_ps(:, 2), ...
                                a(1), ...
                                a(2), ...
                                a(3), ...
                                d_cell{:});

    % If derivative is a constant, sometimes the output is a single value;
    % if this is the case, repmat until the size is equal to the size of
    % p_ps.
    if ~isequal(size(dp_p_d_dx_p), size(p_ps))
        dp_p_d_dx_p = repmat(dp_p_d_dx_p, size(p_ps, 1), 1);
    end
    if ~isequal(size(dp_p_d_dy_p), size(p_ps))
        dp_p_d_dy_p = repmat(dp_p_d_dy_p, size(p_ps, 1), 1);
    end

    % TODO: find a better way to construct sparse diagonal jacobian,
    % as this is probably the bottleneck

    % Jacobian
    jacob_cell = mat2cell(sparse(reshape(vertcat(dp_p_d_dx_p', dp_p_d_dy_p'), 2, [])), 2, 2*ones(1, size(p_ps, 1)));
    jacob = blkdiag(jacob_cell{:});
end
