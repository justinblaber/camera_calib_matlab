function y = safe_mvnpdf(varargin)
    % Wrapper for mvnpdf to handle cases where inputs cause an error.
    %
    % Inputs:
    %   varargin - cell; inputs to mvnpdf()
    %
    % Outputs:
    %   y - cell; output of mvnpdf()

    % Initialize
    safe = true;

    % Get arguments
    X = varargin{1}; % First argument is always X

    % Check if arguments are not safe
    if isempty(X)
        safe = false;
    elseif nargin == 3
        SIGMA = varargin{3}; % Third argument is always SIGMA

        % Make sure SIGMA is positive definite
        if ~alg.is_pos_def(SIGMA)
            safe = false;
        end
    end

    % Call function if its safe
    if safe
        y = mvnpdf(varargin{:});
    else
        % Output nans
        y = nan(size(X, 1), 1);
    end
end
