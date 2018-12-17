function Aq = ellipse2conic(e)
    % Converts input ellipse parameters to conic matrix.
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
    %   Aq - array; 3x3 conic matrix matrix stored as:
    %       [A   B/2 D/2;
    %        B/2 C   E/2;
    %        D/2 E/2 F];

    % Get parameters
    h = e(1);
    k = e(2);
    a = e(3);
    b = e(4);
    alpha = e(5);

    % Get conic polyinomial coefficients; from wikipedia: https://en.wikipedia.org/wiki/Ellipse
    A = a^2*sin(alpha)^2 + b^2*cos(alpha)^2;
    B = 2*(b^2 - a^2)*sin(alpha)*cos(alpha);
    C = a^2*cos(alpha)^2 + b^2*sin(alpha)^2;
    D = -2*A*h - B*k;
    E = -B*h - 2*C*k;
    F = A*h^2 + B*h*k + C*k^2 - a^2*b^2;

    % Get conic matrix
    Aq = [A   B/2 D/2;
          B/2 C   E/2;
          D/2 E/2 F];
end
