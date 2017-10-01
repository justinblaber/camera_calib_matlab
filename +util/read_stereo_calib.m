function [cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,R_s,t_s,homographies_refine,calib_config] = read_stereo_calib(file_path)
    % Reads stereo calibration file
    %
    % Inputs:
    %   file_path - string; path to file to read calibration output from
    %
    % Outputs:
    %   cb_imgs - struct; contains:
    %       .L - class.img; calibration board images for the left camera
    %       .R - class.img; calibration board images for the right camera
    %   board_points_ps - struct; contains:
    %       .L - cell; cell array of calibration board points for the left 
    %           camera.
    %       .R - cell; cell array of calibration board points for the right
    %           camera.
    %   four_points_ps - struct; contains:
    %       .L - cell; cell array of four points for the left camera
    %       .R - cell; cell array of four points for the right camera
    %   A - struct; contains:
    %       .L - array; camera matrix for the left camera
    %       .R - array; camera matrix for the right camera
    %   distortion - struct; contains:
    %       .L - array; 4x1 array of distortions for the left camera
    %       .R - array; 4x1 array of distortions for the right camera
    %       stored as: 
    %           [beta1; beta2; beta3; beta4]  
    %   rotations - struct; contains:
    %       .L - cell; rotations for the left camera
    %       .R - cell; rotations for the right camera
    %   translations - struct; contains:
    %       .L - cell; translations for the left camera
    %       .R - cell; translations for the right camera
    %   R_s - array; 3x3 rotation matrix describing rotation from the left
    %       camera to the right camera
    %   t_s - array; 3x1 translation vector describing translation from the
    %       left camera to the right camera
    %   homographies_refine - struct; contains:
    %       .L - cell; cell array of homographies used for subpixel 
    %           checkerboard corner refinement for the left camera image. 
    %           Used for debugging.
    %       .R - cell; cell array of homographies used for subpixel 
    %           checkerboard corner refinement for the right camera image. 
    %           Used for debugging.
    %   calib_config - struct; this is the struct returned by
    %       util.load_calib_config()
                   
    % TODO: add checks to validate file
    
    % Check to make sure data file exists
    if exist(file_path,'file') == 0
        error(['Calibration file: ' file_path ' does not exist.']);
    end
    
    % Read data
    calib = util.read_data(file_path);
    
    % Parse out left
    [cb_imgs.L,board_points_ps.L,four_points_ps.L,A.L,distortion.L,rotations.L,translations.L,homographies_refine.L,calib] = util.parse_single_calib(calib,'_L');
    
    % Parse out right
    [cb_imgs.R,board_points_ps.R,four_points_ps.R,A.R,distortion.R,rotations.R,translations.R,homographies_refine.R,calib] = util.parse_single_calib(calib,'_R');
    
    % Read R_s and t_s
    [R_s,calib] = read_and_remove(calib,'R_s');
    [t_s,calib] = read_and_remove(calib,'t_s');
    
    % The remainder is calib_config
    calib_config = calib;
end

function [param, calib] = read_and_remove(calib,field)
    param = calib.(field);
    calib = rmfield(calib,field);
end
