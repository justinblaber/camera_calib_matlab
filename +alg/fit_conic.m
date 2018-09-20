function Aq = fit_conic(array_dx,array_dy)
    % Given an input array containing a conic section, this will attempt to
    % find the parameters of the conic matrix.
    %
    % Inputs:
    %   array_dx - array; MxN array gradient in x direction
    %   array_dy - array; MxN array gradient in y direction
    %
    % Outputs:
    %   Aq - array; 3x3 conic matrix matrix stored as:
    %       [A   B/2 D/2;
    %        B/2 C   E/2;
    %        D/2 E/2 F];

    if ~isequal(size(array_dx),size(array_dy))
        error('Input gradient arrays must be equal in size');
    end
        
    % Get coordinates of each pixel
    [y,x] = ndgrid(1:size(array_dx,1),1:size(array_dx,2));
    p = [x(:) y(:)];

    % Normalize coordinates; shift the origin of the coordinate system to
    % the center of the image, and then scale the axes such that the mean
    % distance to the center becomes sqrt(2); this should be "good enough".
    T = norm_mat(p);
    p_aug = [p ones(size(p,1),1)]';
    p_norm = T*p_aug;
    
    % Get homogeneous coordinates of lines
    l = [array_dx(:) array_dy(:) -(array_dx(:).*p_norm(1,:)' + array_dy(:).*p_norm(2,:)')];

    % Form linear equations and solve for inverse conic
    A = [l(:,1).^2 l(:,1).*l(:,2) l(:,2).^2 l(:,1).*l(:,3) l(:,2).*l(:,3)];
    b = -l(:,3).^2;

    % Solve
    aq = lscov(A,b);

    % Get conic matrix
    Aq_inv = [aq(1)   aq(2)/2 aq(4)/2;
              aq(2)/2 aq(3)   aq(5)/2;
              aq(4)/2 aq(5)/2 1];
    Aq = inv(Aq_inv);
    
    % Rescale conic matrix to take normalization into account
    Aq = T'*Aq*T; %#ok<MINV>
end

function T_norm = norm_mat(points)
    points_x = points(:,1);
    points_y = points(:,2);
    mean_x = mean(points_x);
    mean_y = mean(points_y);    
    sm = sqrt(2)*size(points,1)./sum(sqrt((points_x-mean_x).^2+(points_y-mean_y).^2));
    T_norm = [sm 0  -mean_x*sm;
              0  sm -mean_y*sm;
              0  0   1];
end
