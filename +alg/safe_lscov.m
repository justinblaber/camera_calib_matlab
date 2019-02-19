function varargout = safe_lscov(varargin)
    % Wrapper for lscov to handle cases where inputs cause an error.
    %
    % Inputs:
    %   varargin - cell; inputs to lscov()
    %
    % Outputs:
    %   varargout - cell; outputs of lscov()

    % Initialize
    safe = true;

    % Get arguments
    A = varargin{1}; % First argument is always A
    B = varargin{2}; % Second argument is always B

    % Check if arguments are not safe
    % Note: NaN check for A hides annoying and wrong rank deficient
    % warning; do not use isfinite() as A may be sparse and isfinite()
    % causes it to fully expand.
    if isempty(A) || any(isnan(A(:)))
        safe = false;
    elseif nargin == 3 || nargin == 4
        w_or_V = varargin{3}; % Third argument is either w or V

        % Check w_or_V:
        %   1) If any values are nans
        %   2) If w_or_V is a sparse covariance matrix which is not
        %       positive definite; this is not supported by lscov()
        if any(isnan(w_or_V(:))) || ...
           (isequal(size(w_or_V), [size(A, 2), size(A, 2)]) && issparse(w_or_V) && ~alg.is_pos_def(w_or_V))
            safe = false;
        end
    end

    % Call function if its safe
    if safe
        [varargout{1:nargout}] = lscov(varargin{:});
    else
        % Output nans
        for i = 1:max(1, nargout)
            switch i
                case 1 % x
                    varargout{i} = nan(size(A, 2), size(B, 2)); %#ok<AGROW>
                case 2 % stdx
                    varargout{i} = nan(size(A, 2), 1); %#ok<AGROW>
                case 3 % mse
                    varargout{i} = nan; %#ok<AGROW>
                case 4 % S
                    varargout{i} = nan(size(A, 2), size(A, 2)); %#ok<AGROW>
                otherwise
                    error('There must only be 1 to 4 output arguments!')
            end
        end
    end
end
