function angles = dominant_grad_angles(array_dx, array_dy, num_angles, opts, W)
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
    %   W - array; optional MxN weight array
    %
    % Outputs:
    %   angles - array; Px1 dominant angles

    if ~exist('W', 'var')
        W = ones(size(array_dx));
    end

    % Get number of bins and space between peaks
    num_bins = opts.dominant_grad_angles_num_bins;
    space_peaks = opts.dominant_grad_angles_space_peaks;

    % Initialize histogram of indices
    hist_idx = zeros(num_bins, 1);

    % Get spacing
    spacing = pi/num_bins;

    % Get "magnitude vector" - apply weights here
    vec_mag = sqrt(array_dx.^2 + array_dy.^2).*W;
    vec_mag = reshape(vec_mag, [], 1);

    % Get "angle vector"
    vec_angle = atan(array_dy./array_dx);             % Between [-pi/2, pi/2]
    vec_angle = reshape(vec_angle, [], 1);

    % Mask out small magnitude values - these can produce NaNs in angle
    % vector and also contribute very little to histogram
    mask = vec_mag < eps('single');
    vec_mag(mask) = [];
    vec_angle(mask) = [];

    % Get "index vector"
    vec_idx = (vec_angle+pi/2+spacing/2)/spacing;     % Between [0.5, num_bins+0.5]

    % Make sure no indices are NaN, which is possible if input gradient
    % arrays contain a nan
    if any(isnan(vec_idx(:)))
        angles = nan(num_angles, 1);
        return
    end

    % Accumulate indices which are integers directly first ---------------%

    % Get integer indices
    idx_int = alg.is_int(vec_idx);

    % Get integer index and magnitude vectors
    vec_idx_int = vec_idx(idx_int);
    vec_mag_int = vec_mag(idx_int);

    % Accumulate
    hist_idx = hist_idx + accumarray(vec_idx_int, vec_mag_int, [num_bins, 1]);

    % Remove integer index values from arrays
    vec_idx(idx_int) = [];
    vec_mag(idx_int) = [];

    % Use linear interpolation to accumulate non-integer indices ---------%

    % Get floored idx and magnitude vectors
    vec_idx_floor = floor(vec_idx);                 % Between [0, num_bins]
    vec_mag_floor = 1-(vec_idx-vec_idx_floor);

    % Get ceiled idx and magnitude vectors
    vec_idx_ceil = ceil(vec_idx);                   % Between [1, num_bins+1]
    vec_mag_ceil = 1-(vec_idx_ceil-vec_idx);

    % Apply mod to idx arrays
    vec_idx_floor = mod(vec_idx_floor-1, num_bins) + 1;
    vec_idx_ceil  = mod(vec_idx_ceil-1,  num_bins) + 1;

    % Apply array_mag to array_mag_floor and array_mag_ceil. Note that sum
    % of array_mag_floor and array_mag_ceil should equal array_mag.
    vec_mag_floor = vec_mag_floor.*vec_mag;
    vec_mag_ceil  = vec_mag_ceil.*vec_mag;

    % Accumulate
    hist_idx = hist_idx + ...
               accumarray(vec_idx_floor, vec_mag_floor, [num_bins, 1]) + ...
               accumarray(vec_idx_ceil,  vec_mag_ceil,  [num_bins, 1]);

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
