function e = conic2ellipse(Aq)
    % Converts input conic matrix into ellipse parameters.
    %
    % Inputs:
    %   Aq - array; 3x3 conic matrix matrix stored as:
    %       [A   B/2 D/2;
    %        B/2 C   E/2;
    %        D/2 E/2 F];
    %
    % Outputs:
    %   e - array; 5x1 ellipse matrix stored as:
    %       e(1) = h; x component of center of ellipse
    %       e(2) = k; y component of center of ellipse
    %       e(3) = a; major axis length 
    %       e(4) = b; minor axis length
    %       e(5) = alpha; rotation of major axis
    
    % Get conic polynomial coefficients
    A = Aq(1,1);
    B = 2*Aq(1,2);
    C = Aq(2,2);
    D = 2*Aq(1,3);
    E = 2*Aq(2,3);
    F = Aq(3,3);
    
    % Make sure input conic is an ellipse
    if abs(B^2-4*A*C) < eps('single') || B^2-4*A*C > 0
        error(['Input conic: "' num2str([A B C D E F]) '" is not an ellipse.']);
    end
    
    % Equations from https://math.stackexchange.com/a/820896/39581
    
    % Get "coefficient of normalizing factor"
    q = 64*(F*(4*A*C-B^2)-A*E^2+B*D*E-C*D^2)/(4*A*C-B^2)^2;
    
    % Get "distance between center and focal point"
    s = 1/4*sqrt(abs(q)*sqrt(B^2+(A-C)^2));

    % Get major axis
    a = 1/8*sqrt(2*abs(q)*sqrt(B^2+(A-C)^2)-2*q*(A+C));
    
    % Get minor axis
    b = sqrt(a^2-s^2);

    % Get center of ellipse
    h = (B*E-2*C*D)/(4*A*C-B^2);
    k = (B*D-2*A*E)/(4*A*C-B^2);
    
    % Get alpha; note that range of alpha is [0,pi)
    if abs(q*A-q*C) < eps('single') && abs(q*B) < eps('single')
        % major and minor axes are equal (i.e. a circle)
        alpha = 0;
    elseif abs(q*A-q*C) < eps('single') && q*B > 0
        % alpha = 1/4*pi
        alpha = 1/4*pi;
    elseif abs(q*A-q*C) < eps('single') && q*B < 0
        % alpha = 3/4*pi
        alpha = 3/4*pi;
    elseif q*A-q*C > 0 && (abs(q*B) < eps('single') || q*B > 0)
        % 0 <= alpha < 1/4*pi
        alpha = 1/2*atan(B/(A-C));
    elseif q*A-q*C > 0 && q*B < 0
        % 3/4*pi < alpha < pi
        alpha = 1/2*atan(B/(A-C)) + pi;
    elseif q*A-q*C < 0
        % 1/4*pi < alpha < 3/4*pi
        alpha = 1/2*atan(B/(A-C)) + 1/2*pi;
    end
    
    % Store ellipse parameters
    e = [h k a b alpha]';
end
