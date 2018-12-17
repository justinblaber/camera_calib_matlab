function verbose_disp(varargin)
    % Will disp text depending on verbosity.
    %
    % Inputs:
    %   varargin - cell;
    %       {1:end-2} - inputs to disp
    %       {end-1} - s; string
    %       {end} - struct;
    %           .verbosity - level of verbosity

    util.verbose_wrapper(@disp, varargin{:});
end
