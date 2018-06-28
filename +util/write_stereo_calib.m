function write_stereo_calib(calib,R_s,t_s,file_path)
    % Writes outputs of stereo calibration to a file, so it can be read
    % again later.
    % 
    % Inputs:
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
    %       left camera to the right cameras
    %   file_path - string; path to file to write calibration output to
    
    % Write left
    util.write_single_calib(calib.L,file_path,'_L');
                        
    % Write right
    util.write_single_calib(calib.R,file_path,'_R',true);
                        
    % Write R_s and t_s    
    util.write_array(R_s,'R_s',file_path);
    util.write_newline(file_path);
    util.write_array(t_s,'t_s',file_path);
    util.write_newline(file_path);
end
