function ps = sample_ellipse(e, theta_samples)
    % Samples ellipse, given ellipse parameters and theta samples
    %
    % Inputs:
    %   e - array; 5x1 ellipse matrix stored as:
    %       e(1) = h; x component of center of ellipse
    %       e(2) = k; y component of center of ellipse
    %       e(3) = a; major axis length
    %       e(4) = b; minor axis length
    %       e(5) = alpha; rotation of major axis
    %   theta_samples - array; Nx1 array of theta samples
    %
    % Outputs:
    %   ps - array; Nx2 array of ellipse points

    ps = [e(3)*cos(e(5))*cos(theta_samples) - e(4)*sin(e(5))*sin(theta_samples) + e(1), ...
          e(3)*sin(e(5))*cos(theta_samples) + e(4)*cos(e(5))*sin(theta_samples) + e(2)];
end
