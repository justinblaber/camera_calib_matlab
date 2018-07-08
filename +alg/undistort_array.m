function array_undistorted = undistort_array(array,A,distortion)
    % This will undistort input array given the camera matrix and 
    % distortion parameters.
    %
    % Inputs:
    %   array - array; MxN array containing distorted image
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]
    %   distortion - array; 4x1 distortions (radial and tangential)
    %       stored as: [beta1; beta2; beta3; beta4]
    %
    % Outputs:
    %   array_undistorted - array; MxN array containing undistorted image
    
    % Set dummy rotation and translation
    R = eye(3);
    t = [0; 0; 1];
    
    % Get "ideal" pixel coordinates based on array size
    [y_p, x_p] = ndgrid(1:size(array,1),1:size(array,2));

    % Convert "ideal" pixel coordinates to world coordinates
    p_bar = [R(:,1:2) t]^-1*A^-1*[x_p(:) y_p(:) ones(numel(y_p),1)]';

    x_w = (p_bar(1,:)./p_bar(3,:))';
    y_w = (p_bar(2,:)./p_bar(3,:))';

    % Get "distorted" pixel coordinates
    p_m = alg.p_m(A, ...
                  distortion, ...
                  R, ...
                  t, ...
                  [x_w y_w]);

    % Resample to undistort array          
    array_undistorted = alg.array_interp(array,p_m,'cubic');        
    array_undistorted = reshape(array_undistorted,size(array,1),size(array,2));
end