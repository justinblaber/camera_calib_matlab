function [four_points_ps,four_points_debugs] = stereo_four_points_detect(cb_imgs,calib_config)
    % Obtains the locations of the four points (fiducial markers) around 
    % the calibration board images.
    % 
    % Inputs:
    %   cb_imgs - struct; contains:
    %       .L and .R - class.img; Mx1 calibration board images
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   four_points_ps - struct; contains:
    %       .L and .R - cell; Mx1 cell array of four points
    %   four_points_debugs - struct; contains:
    %       .L and .R - struct; Mx1 struct of four point debugging info
    
    disp('------');
    disp('Performing four-point detection for left images...');
    [four_points_ps.L,four_points_debugs.L] = alg.single_four_points_detect(cb_imgs.L,calib_config);
    disp('------');
    disp('Performing four-point detection for right images...');
    [four_points_ps.R,four_points_debugs.R] = alg.single_four_points_detect(cb_imgs.R,calib_config);
end
