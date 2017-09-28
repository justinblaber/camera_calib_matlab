function write_stereo_calib(cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,R_s,t_s,homographies_refine,cal_config,file_path)
    % Writes outputs of stereo calibration to a file, so it can be read
    % again later.
    % 
    % Inputs:
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
    %   cal_config - struct; this is the struct returned by
    %       util.load_cal_config()
    %   file_path - string; path to file to write calibration output to
    
    % Write left
    util.write_single_calib(cb_imgs.L, ...
                            board_points_ps.L, ...
                            four_points_ps.L, ...
                            A.L, ...
                            distortion.L, ...
                            rotations.L, ...
                            translations.L, ...
                            homographies_refine.L, ...
                            cal_config, ...
                            file_path, ...
                            '_L');
                        
    % Write right
    util.write_single_calib(cb_imgs.R, ...
                            board_points_ps.R, ...
                            four_points_ps.R, ...
                            A.R, ...
                            distortion.R, ...
                            rotations.R, ...
                            translations.R, ...
                            homographies_refine.R, ...
                            cal_config, ...
                            file_path, ...
                            '_R', ...
                            true);
                        
    % Write R_s and t_s    
    util.write_array(R_s,'R_s',file_path);
    util.write_newline(file_path);
    util.write_array(t_s,'t_s',file_path);
    util.write_newline(file_path);
end