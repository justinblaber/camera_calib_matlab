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
    
    % Rotation matrix must have a determinant of positive 1
    if round(det(R)) ~= 1 % round to account for double precision errors
        error(['Rotation matrix approximation does not have a ' ...
               'determinant of 1!'])
    end
end
