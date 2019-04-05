function varargout = safe_svd(varargin)
    % Wrapper for svd to handle cases where inputs cause an error.
    %
    % Inputs:
    %   varargin - cell; inputs to svd()
    %
    % Outputs:
    %   varargout - cell; outputs of svd()

    % Initialize
    safe = true;

    % Get arguments
    A = varargin{1}; % First argument is always A

    % Check if arguments are not safe
    if any(~isfinite(A(:)))
        safe = false;
    end

    % Call function if its safe
    if safe
        [varargout{1:nargout}] = svd(varargin{:});
    else
        % Output nans
        for i = 1:max(1, nargout)
            switch i
                case 1 % U
                    varargout{i} = nan(size(A, 1), size(A, 1)); %#ok<AGROW>
                case 2 % S
                    varargout{i} = nan(size(A)); %#ok<AGROW>
                case 3 % V
                    varargout{i} = nan(size(A, 2), size(A, 2)); %#ok<AGROW>
                otherwise
                    error('There must only be 1 to 3 output arguments!')
            end
        end
    end
end
