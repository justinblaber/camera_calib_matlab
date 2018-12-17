function jacob = dh_drt(A)
    % This will compute the jacobian of the homography wrt "rt".
    %
    % Inputs:
    %   A - array; 3x3 camera matrix
    %
    % Outputs:
    %   jacob - array; 9x12 array.
    %       Format of jacobian is:
    %
    %            dr11 dr21 dr31 dr12 dr22 dr32 dr13 dr23 dr33 dtx dty dtz
    %       dh11
    %       dh21
    %       dh31
    %       dh12
    %       dh22
    %       dh32
    %       dh13
    %       dh23
    %       dh33

    jacob = [A        zeros(3) zeros(3) zeros(3);
             zeros(3) A        zeros(3) zeros(3);
             zeros(3) zeros(3) zeros(3) A];
end
