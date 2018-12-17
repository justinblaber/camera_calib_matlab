function verbose_fprintf(varargin)
    % Will fprintf text depending on verbosity.
    %
    % Inputs:
    %   varargin - cell;
    %       {1:end-2} - inputs to fprintf
    %       {end-1} - s; string
    %       {end} - struct;
    %           .verbosity - level of verbosity

    util.verbose_wrapper(@fprintf, varargin{:});
end
