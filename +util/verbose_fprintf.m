function verbose_fprintf(varargin)
    % Will fprintf text depending on verbosity.
    %
    % Inputs:
    %   varargin - cell;
    %       {1:end-2} - inputs to function
    %       {end-1} - int; verbose input
    %       {end} - struct;
    %           .verbosity - int; level of verbosity

    util.verbose_wrapper(@fprintf, varargin{:});
end
