function blobs = blob_detect_thresh(array, opts)
    % Performs blob detection on input array; this returns dark blobs.
    %
    % Inputs:
    %   array - array; MxN array
    %   opts - struct;
    %       .blob_detect_thresh_range1 - scalar; starting point for
    %           thresholding
    %       .blob_detect_thresh_range2 - scalar; end point for threshold
    %       .blob_detect_thresh_step - scalar; increment of threshold
    %       .blob_detect_thresh_solidity_cutoff - scalar; cutoff for
    %           "solidity"
    %       .blob_detect_thresh_area_cutoff - scalar; cutoff for area
    %       .blob_detect_thresh_majoraxis_cutoff - scalar; cutoff for major
    %           axis
    %       .blob_detect_thresh_minoraxis_cutoff - scalar; cutoff for minor
    %           axis
    %       .blob_detect_thresh_d_cluster - scalar; clusters blobs within
    %           d distance.
    %       .blob_detect_thresh_r1_cluster - scalar; clusters blobs within
    %           r1 distance.
    %       .blob_detect_thresh_r2_cluster - scalar; clusters blobs within
    %           r2 distance.
    %
    % Outputs:
    %   blobs - array; Px5 array containing:
    %       blobs(i, 1) = h; x component of center of blob
    %       blobs(i, 2) = k; y component of center of blob
    %       blobs(i, 3) = a; major axis length
    %       blobs(i, 4) = b; minor axis length
    %       blobs(i, 5) = alpha; rotation of major axis

    % Get bounding box of array
    bb_array = alg.bb_array(array);

    % Normalize array
    array = alg.normalize_array(array, 'min-max');

    % Initialize blobs
    blobs = zeros(0, 5);

    % Get thresholds
    threshes = opts.blob_detect_thresh_range1: ...
               opts.blob_detect_thresh_step: ...
               opts.blob_detect_thresh_range2;

    % Sort thresholds such that the middle is applied first and outer
    % thresholds are applied last; I'm assuming middle threshold is the
    % best guess
    thresh_dist = abs(threshes - (opts.blob_detect_thresh_range2 + opts.blob_detect_thresh_range1)/2);
    [~, thresh_idx] = sort(thresh_dist);

    % Iterate over threshold
    for i = 1:numel(thresh_idx)
        % Get thresh
        thresh = threshes(thresh_idx(i));

        % Get regionprops
        rps = regionprops(bwconncomp(array < thresh, 4), ... % Do 4 way connection
                          'Centroid', ...
                          'Solidity', ...
                          'Area', ...
                          'MajorAxisLength', ...
                          'MinorAxisLength', ...
                          'Orientation');

        % Apply solidity cutoff
        rps = rps([rps.Solidity] >= opts.blob_detect_thresh_solidity_cutoff);

        % Apply area cutoff
        rps = rps([rps.Area] >= opts.blob_detect_thresh_area_cutoff);

        % Apply major axis cutoff
        rps = rps([rps.MajorAxisLength] >= opts.blob_detect_thresh_majoraxis_cutoff);

        % Apply minor axis cutoff
        rps = rps([rps.MinorAxisLength] >= opts.blob_detect_thresh_minoraxis_cutoff);

        % Iterate over blobs
        for j = 1:numel(rps)
            % Get blob
            blob = [rps(j).Centroid, ...
                    rps(j).MajorAxisLength/2, ...
                    rps(j).MinorAxisLength/2, ...
                    -rps(j).Orientation/180*pi];

            % Make sure bounds are in array
            bb_blob = alg.bb_ellipse(blob);
            if alg.is_bb_in_bb(bb_blob, bb_array)
                % Do some very rudimentary clustering
                dist_d = sqrt((blobs(:, 1)-blob(1)).^2 + (blobs(:, 2)-blob(2)).^2);
                dist_r1 = abs(blobs(:, 3)-blob(3));
                dist_r2 = abs(blobs(:, 4)-blob(4));
                if all(dist_d > opts.blob_detect_thresh_d_cluster | ...
                       dist_r1 > opts.blob_detect_thresh_r1_cluster | ...
                       dist_r2 > opts.blob_detect_thresh_r2_cluster)
                    % Append to list
                    blobs(end + 1, :)= blob; %#ok<AGROW>
                end
            end
        end
    end
end
