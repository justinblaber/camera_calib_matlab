function verbose_wrapper(f, varargin)
    % Will only call f if verbosity level is met.
    %
    % Inputs:
    %   f - function handle;
    %   varargin - cell;
    %       {1:end-2} - inputs to function
    %       {end-1} - s; string
    %       {end} - struct;
    %           .verbosity - level of verbosity

    if varargin{end}.verbosity >= varargin{end-1}
        f(varargin{1:end-2});
    end
end
