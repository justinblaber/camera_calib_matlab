function xform = ellipse2xform(e)
    % Converts input ellipse to an affine transformation.
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
    %   xform - array; 3x3 affine matrix which converted unit circle to
    %       ellipse
    
    % Get scaling
    s = [e(3) 0; ...
         0    e(4)];
    
    % Get rotation
    R = [cos(e(5)) -sin(e(5));  ...
         sin(e(5))  cos(e(5))];
     
    % Create affine transform
    xform = [R*s        [e(1); e(2)];
             zeros(1,2) 1];
end