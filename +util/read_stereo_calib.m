function [calib,R_s,t_s] = read_stereo_calib(file_path)
    % Reads a stereo calibration file
    %
    % Inputs:
    %   file_path - string; path to file to read calibration
    %
    % Outputs:
    %   calib - struct; contains:
    %       .L and .R - struct; contains:
    %           .config - struct; this is the struct returned by
    %               util.read_calib_config()
    %           .intrin.A - array; optimized camera matrix
    %           .intrin.distortion - array; optimized distortions (radial 
    %               and tangential) stored as: 
    %               [beta1; beta2; beta3; beta4]  
    %           .extrin(i).cb_img - class.img; ith calibration board image
    %           .extrin(i).rotation - array; ith optimized rotation
    %           .extrin(i).translation - array; ith optimized translation
    %           .extrin(i).four_points_p - array; ith array of four points
    %               around calibration board in pixel coordinates.
    %           .extrin(i).board_points_p - array; ith array of optimized 
    %               subpixel calibration board points in pixel coordinates.
    %           .extrin(i).debug.homography_refine - array; ith homography 
    %               used for subpixel target refinement.
    %   R_s - array; optimized rotation describing rotation from the left 
    %       camera to the right camera
    %   t_s - array; optimized translation describing translation from the 
    %       left camera to the right camera
                   
    % TODO: add checks to validate file
    
    % Check to make sure data file exists
    if exist(file_path,'file') == 0
        error(['Calibration file: ' file_path ' does not exist.']);
    end
    
    % Read data
    calib_raw = util.read_data(file_path);
    
    % Parse out the left calibration
    [calib.L, calib_raw] = util.parse_single_calib(calib_raw,'_L');
    
    % Parse out the right calibration
    [calib.R, calib_raw] = util.parse_single_calib(calib_raw,'_R');
    
    % Parse R_s and t_s
    [R_s, calib_raw] = read_and_remove(calib_raw,'R_s');
    [t_s, calib_raw] = read_and_remove(calib_raw,'t_s');
    
    % The remainder should be calib_config
    calib.L.config = calib_raw;
    calib.R.config = calib_raw;
end

function [param, calib_raw] = read_and_remove(calib_raw,field)
    param = calib_raw.(field);
    calib_raw = rmfield(calib_raw,field);
end
