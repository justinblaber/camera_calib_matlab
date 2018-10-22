function [cov,p] = ellipse2cov(e)
    % Converts input ellipse to a covariance matrix and point
    %
    % Inputs:
    %   e - array; 5x1 ellipse matrix stored as:
    %       e(1) = h; x component of center of ellipse
    %       e(2) = k; y component of center of ellipse
    %       e(3) = a; major axis length 
    %       e(4) = b; minor axis length
    %       e(5) = alpha; rotation of major axis
    %
    % Outputs:
    %   cov - array; 2x2 covariance array
    %   p - array; 1x2 point
    
    % Get point
    p = [e(1) e(2)];
    
    % Get covariance matrix (set variance to major and minor axes, then
    % rotate)
    var = [e(3)^2 0; ...
           0      e(4)^2];
    R = [cos(e(5)) -sin(e(5));  ...
         sin(e(5))  cos(e(5))];
    cov = R*var*R';
end