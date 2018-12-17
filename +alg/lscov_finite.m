function varargout = lscov_finite(varargin)
    % Wrapper for lscov to handle case where weights are not finite or if
    % covariance matrix is sparse and not positive definite.
    %
    % Inputs:
    %   varargin - cell; inputs to lscov()
    %
    % Outputs:
    %   varargout - cell; outputs of lscov()

    % Make sure weights or covariance matrix is input
    if nargin >= 3
        A = varargin{1};        % First argument is always A
        B = varargin{2};        % Second argument is always B
        w_or_V = varargin{3};   % Third argument is either w or V

        % Check w_or_V:
        %   1) If any values are not finite
        %   2) If w_or_V is a sparse covariance matrix which is not
        %       positive definite; this is not supported by lscov()
        if any(~isfinite(w_or_V(:))) || ...
           (isequal(size(w_or_V), [size(A, 2), size(A, 2)]) && issparse(w_or_V) && ~alg.is_pos_def(w_or_V))
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

            % Return
            return
        end
    end

    % Otherwise, call lscov normally
    [varargout{1:nargout}] = lscov(varargin{:});
end
