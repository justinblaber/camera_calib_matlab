function y = mvnpdf_pos_def(varargin)
    % Wrapper for mvnpdf to handle case where SIGMA is not positive
    % definite.
    %
    % Inputs:
    %   varargin - cell; inputs to mvnpdf()
    %
    % Outputs:
    %   y - cell; output of mvnpdf()
    
    % Make sure covariance array is input
    if nargin >= 3
        X = varargin{1};     % First argument is always X
        SIGMA = varargin{3}; % Third argument is always SIGMA
        
        % Make sure SIGMA is positive definite
        if ~alg.is_pos_def(SIGMA)
            % Output nans
            y = nan(size(X,1),1);
            
            % Return
            return
        end
    end
    
    % Otherwise, call mvnpdf normally
    y = mvnpdf(varargin{:});
end