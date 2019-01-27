function angles = dominant_grad_angles(array_dx, array_dy, num_angles, opts)
    % Returns dominant angles found in input gradient array using hough
    % transform.
    %
    % Inputs:
    %   array_dx - array; MxN array gradient in x direction
    %   array_dy - array; MxN array gradient in y direction
    %   num_angles - int; number of angles returned
    %   opts - struct;
    %       .dominant_grad_angles_num_bins - int; number of bins used in
    %           hough transform
    %       .dominant_grad_angles_space_peaks - int; space between peaks
    %
    % Outputs:
    %   angles - array; Px1 dominant angles

    % Make sure gradients are equal in size
    if ~isequal(size(array_dx), size(array_dy))
        error('Input gradient arrays must be equal in size');
    end

    % Get number of bins and space between peaks
    num_bins = opts.dominant_grad_angles_num_bins;
    space_peaks = opts.dominant_grad_angles_space_peaks;

    % Initialize histogram of indices
    hist_idx = zeros(num_bins, 1);

    % Get spacing
    spacing = pi/num_bins;

    % Get magnitude array
    array_mag = sqrt(array_dx.^2 + array_dy.^2);

    % Get angle array
    array_angle = atan(array_dy./array_dx);             % Between [-pi/2, pi/2]

    % Get only finite values; note this converts arrays to column vectors
    mask = isfinite(array_mag) & isfinite(array_angle);
    array_mag = array_mag(mask);
    array_angle = array_angle(mask);

    % Get "index array"
    array_idx = (array_angle+pi/2+spacing/2)/spacing;   % Between [0.5, num_bins+0.5]

    % Accumulate indices which are integers directly first ---------------%

    % Get integer indices
    idx_int = alg.is_int(array_idx);

    % Get integer index and magnitude arrays
    array_idx_int = array_idx(idx_int);
    array_mag_int = array_mag(idx_int);

    % Accumulate
    hist_idx = hist_idx + accumarray(array_idx_int, array_mag_int, [num_bins, 1]);

    % Remove integer index values from arrays
    array_idx(idx_int) = [];
    array_mag(idx_int) = [];

    % Use linear interpolation to accumulate non-integer indices ---------%

    % Get floored idx and magnitude arrays
    array_idx_floor = floor(array_idx);                 % Between [0, num_bins]
    array_mag_floor = 1-(array_idx-array_idx_floor);

    % Get ceiled idx and magnitude arrays
    array_idx_ceil = ceil(array_idx);                   % Between [1, num_bins+1]
    array_mag_ceil = 1-(array_idx_ceil-array_idx);

    % Apply mod to idx arrays
    array_idx_floor = mod(array_idx_floor-1, num_bins) + 1;
    array_idx_ceil  = mod(array_idx_ceil-1,  num_bins) + 1;

    % Apply array_mag to array_mag_floor and array_mag_ceil. Note that sum
    % of array_mag_floor and array_mag_ceil should equal array_mag.
    array_mag_floor = array_mag_floor.*array_mag;
    array_mag_ceil  = array_mag_ceil.*array_mag;

    % Accumulate
    hist_idx = hist_idx + ...
               accumarray(array_idx_floor, array_mag_floor, [num_bins, 1]) + ...
               accumarray(array_idx_ceil,  array_mag_ceil,  [num_bins, 1]);

    % Get peaks ----------------------------------------------------------%

    % TODO: possibly smooth hist_idx first

    idx_peaks = zeros(num_angles, 1);
    for i = 1:num_angles
        % Find peak
        [~, idx_peak] = max(hist_idx);

        % Get points for Gauss Newton refinement
        idx_GN = idx_peak-1:idx_peak+1;
        p_GN = hist_idx(mod(idx_GN-1, num_bins)+1);

        % Set adjacent values to zero so next peak isnt too close to this one
        hist_idx(mod([idx_peak-space_peaks:idx_peak+space_peaks]-1, num_bins)+1) = 0; %#ok<NBRAK>

        % Do Gauss Newton update
        idx_peak = idx_peak - ((p_GN(3)-p_GN(1))/2)/(p_GN(3)-2*p_GN(2)+p_GN(1));

        % Store peak
        idx_peaks(i) = idx_peak;
    end

    % Convert peak indices to angles -------------------------------------%

    angles = spacing*(idx_peaks-1/2)-pi/2;
end
