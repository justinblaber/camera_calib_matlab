function [four_points_ps,four_points_debugs] = single_four_points_detect(cb_imgs,calib_config)
    % Obtains the locations of the four points (fiducial markers) around 
    % the calibration board images.
    % 
    % Inputs:
    %   cb_imgs - class.img; Mx1 calibration board images
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   four_points_ps - cell; Mx1 cell array of four points
    %   four_points_debugs - struct; contains debugging info for four
    %       point detector
    
    disp('---');
    
    four_points_ps = {};
    four_points_debugs = struct('blobs',{},'ellipses',{},'patch_matches',{});
    for i = 1:length(cb_imgs)
        t = tic;
        fprintf('Performing four-point detection for image: %s. ', cb_imgs(i).get_path());
        [four_points_ps{i},four_points_debugs(i)] = alg.four_points_detect(cb_imgs(i).get_array(),calib_config); %#ok<AGROW>
        time = toc(t);
        fprintf(['Time ellapsed: %f seconds.' newline],time);
    end
end
