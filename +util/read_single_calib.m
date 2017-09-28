function [cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,homographies_refine,cal_config] = read_single_calib(file_path)
    % Reads single calibration file
    %
    % Inputs:
    %   file_path - string; path to file to read calibration output from
    %
    % Outputs:
    %   cb_imgs - class.img; calibration board images
    %   board_points_ps - cell; Mx1 cell array of calibration board points 
    %   four_points_ps - cell; Mx1 cell array of four points
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]    
    %   distortion - array; 4x1 array of distortions (radial and 
    %       tangential) stored as: 
    %       [beta_1; beta_2; beta_3; beta_4]
    %   rotations - cell; Mx1 cell array containing 3x3 rotation matrices
    %   translations - cell; Mx1 cell array containing 3x1 translation
    %       vectors
    %   homographies_refine - cell; cell array of homographies used for
    %       subpixel checkerboard corner refinement. Used for debugging.
    %   cal_config - struct; this is the struct returned by
    %       util.load_cal_config()
                
    % TODO: add checks to validate file
    
    % Check to make sure data file exists
    if exist(file_path,'file') == 0
        error(['Calibration file: ' file_path ' does not exist.']);
    end
    
    % Read data
    calib = util.read_data(file_path);
    
    % Parse out the single calibration
    [cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,homographies_refine,cal_config] = util.parse_single_calib(calib);
end