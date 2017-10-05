function calib = read_single_calib(file_path)
    % Reads a single calibration file
    %
    % Inputs:
    %   file_path - string; path to file to read calibration
    %
    % Outputs:
    %   calib - struct; contains:
    %       .config - struct; this is the struct returned by
    %           util.read_calib_config()
    %       .intrin.A - array; optimized camera matrix
    %       .intrin.distortion - array; optimized distortions (radial and
    %           tangential) stored as: 
    %           [beta1; beta2; beta3; beta4]  
    %       .extrin(i).cb_img - class.img; ith calibration board image
    %       .extrin(i).rotation - array; ith optimized rotation
    %       .extrin(i).translation - array; ith optimized translation
    %       .extrin(i).four_points_p - array; ith array of four points
    %           around calibration board in pixel coordinates.
    %       .extrin(i).board_points_p - array; ith array of optimized 
    %           subpixel calibration board points in pixel coordinates.
    %       .extrin(i).debug.homography_refine - array; ith homography used
    %           for subpixel checkerboard corner refinement.
                
    % TODO: add checks to validate file
    
    % Check to make sure data file exists
    if exist(file_path,'file') == 0
        error(['Calibration file: ' file_path ' does not exist.']);
    end
    
    % Read data
    calib_raw = util.read_data(file_path);
    
    % Parse out the single calibration
    [calib, calib_config] = util.parse_single_calib(calib_raw);
    calib.config = calib_config;
end
