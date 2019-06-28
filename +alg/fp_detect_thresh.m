function [p_fps, debug] = fp_detect_thresh(array, opts)
    % Obtains the locations of the four points (fiducial markers) around
    % the calibration board.
    %
    % Inputs:
    %   array - array; MxN array
    %   opts - struct;
    %       .blob_detect_thresh_* - options for alg.blob_detect_thresh()
    %       .ellipse_detect_* - options for alg.fp_detect_blobs()
    %       .fp_detect_* - options for alg.fp_detect_blobs()
    %
    % Outputs:
    %   p_fps - array; 4x2 array of four points
    %   debug - struct;

    % Get blobs
    blobs = alg.blob_detect_thresh(array, opts);

    % Get four points
    [p_fps, debug] = alg.fp_detect_blobs(array, blobs, opts);
end
