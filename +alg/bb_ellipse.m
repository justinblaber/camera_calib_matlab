function bb = bb_ellipse(e)
    % Returns bounding box of ellipse
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
    %   bb - array; 2x2 bounding box in the form:
    %       [top-left point;
    %        bottom-right point]

    height = 2*sqrt(e(3)^2*sin(e(5))^2 + e(4)^2*cos(e(5))^2);
    width = 2*sqrt(e(3)^2*cos(e(5))^2 + e(4)^2*sin(e(5))^2);

    bb = [e(1)-width/2 e(2)-height/2;
          e(1)+width/2 e(2)+height/2];
end

