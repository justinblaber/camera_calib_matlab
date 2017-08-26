function R = approx_rot(R)
    % Gets the "best" (smallest frobenius norm) approximation of the input 
    % matrix as a rotation matrix, subject to R'R = I. Taken from Zhang's 
    % camera calibration paper.
    %
    % Inputs:
    %   R - array; 3x3 matrix
    %
    % Outputs:
    %   R - array; "best" 3x3 rotation matrix approximation of input R
       
    [U,~,V] = svd(R);
    R = U*V';
end