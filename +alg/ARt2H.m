function H = ARt2H(A, R, t)
    % Computes homography from camera matrix, rotation, and translation.
    %
    % Inputs:
    %   A - array; 3x3 camera matrix
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector
    %
    % Outputs:
    %   H - array; 3x3 homography

    H = A*[R(:, 1) R(:, 2) t];
end
