function verbose_disp(varargin)
    % Will disp text depending on verbosity.
    %
    % Inputs:
    %   varargin - cell;
    %       {1:end-2} - inputs to function
    %       {end-1} - int; verbose input
    %       {end} - struct;
    %           .verbosity - int; level of verbosity

    util.verbose_wrapper(@disp, varargin{:});
end
