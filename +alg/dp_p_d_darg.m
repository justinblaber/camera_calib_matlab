function jacob = dp_p_d_darg(p_ps,f_dp_p_d_darg,A,d)
    % This will compute the jacobian of distorted pixel points wrt one of
    % its arguments.
    %
    % Inputs:
    %   p_ps - array; Nx2 array of pixel points
    %   f_dp_p_d_darg - function handle; derivative of p_p2p_p_d wrt an
    %       argument
    %   A - array; 3x3 array containing:
    %       [alpha    0       x_o;
    %        0        alpha   y_o;
    %        0        0       1]
    %   d - array; Mx1 array of distortion coefficients
    % 
    % Outputs:
    %   jacob - sparse array; 2*Nx1 array.
    %       Format of jacobian is:
    %
    %                darg
    %       dx_p_d_1
    %       dy_p_d_1
    %          .
    %          .
    %          .
    %       dx_p_d_N
    %       dy_p_d_N
    
    % Compute partial derivatives
    jacob = alg.p_p2p_p_d(p_ps, ...
                          f_dp_p_d_darg, ...
                          A, ...
                          d);
    
    % If derivative is a constant, sometimes the output is a single value;
    % if this is the case, repmat until the size is equal to the size of
    % p_ps.
    if ~isequal(size(jacob),size(p_ps))
        jacob = repmat(jacob,size(p_ps,1),1);
    end
    
    % Reshape so jacobian is in the desired output format
    jacob = reshape(jacob',[],1);
end