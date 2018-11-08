function [l1, l2] = asymptotes(Aq)
    % Given a conic matrix of a hyperbola, this will return the asymptotes.
    %
    % Inputs:
    %   Aq - array; 3x3 conic matrix matrix stored as:
    %       [A   B/2 D/2;
    %        B/2 C   E/2;
    %        D/2 E/2 F];
    %
    % Outputs:
    %   l1 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0
    %   l2 - array; 3x1 array of a line in the form: 
    %       [a; b; c] where ax + by + c = 0

    % Get conic polynomial coefficients
    A = Aq(1,1);
    B = 2*Aq(1,2);
    C = Aq(2,2);
    D = 2*Aq(1,3); %#ok<NASGU>
    E = 2*Aq(2,3); %#ok<NASGU>
    F = Aq(3,3); %#ok<NASGU>
    
    % Make sure input conic is a hyperbola
    if abs(B^2-4*A*C) < eps('single') || B^2-4*A*C < 0
        l1 = nan(3,1);
        l2 = nan(3,1);
        return
    end
    
    % TODO: handle degenerate case where A is zero
    
    % Get asymptotes
    center = inv(Aq)*[0; 0; 1]; %#ok<MINV>
    r1 = A;
    r2 = 1;
    s1 = B/2 - (B^2 - 4*A*C)^(1/2)/2;
    s2 = (B + (B^2 - 4*A*C)^(1/2))/(2*A);   
    
    % Return lines
    l1 = [r1 s1 -r1*center(1)-s1*center(2)]';
    l2 = [r2 s2 -r2*center(1)-s2*center(2)]';
end
      
