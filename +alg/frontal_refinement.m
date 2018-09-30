function board_points_p_refined = frontal_refinement(array,A,distortion,R,t,calib_config)
    % This will project calibration image into "frontal space" (i.e world
    % coordinates), detect targets in that space, then project these points
    % back into distorted pixel coordinates. These points can be used to 
    % recompute model parameters in an iterative manner. 
    % 
    % From: 
    %     "Accurate Camera Calibration using Iterative Refinement of Control Points"
    %
    % Inputs:    
    %   array - array; calibration board image array
    %	A - array; camera matrix containing:
    %           [alpha    0       x_o;
    %            0        alpha   y_o;
    %            0        0       1]
    %   distortion - array; distortions (radial and tangential) stored as: 
    %       [beta1; beta2; beta3; beta4]  
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation vector
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %	board_points_p_refined - array; Nx2 array of refined points
    
    % Set number of targets in frontal array
    num_targets_frontal_width = calib_config.num_targets_width + 2*calib_config.frontal_refinement_num_targets_padding;
    num_targets_frontal_height = calib_config.num_targets_height + 2*calib_config.frontal_refinement_num_targets_padding;

    % Get frontal array size
    array_frontal_width = num_targets_frontal_width * calib_config.frontal_refinement_pix_per_target + 1;
    array_frontal_height = num_targets_frontal_height * calib_config.frontal_refinement_pix_per_target + 1;

    % Get frontal height and width in world coordinates
    array_frontal_width_w = num_targets_frontal_width * calib_config.target_spacing;
    array_frontal_height_w = num_targets_frontal_height * calib_config.target_spacing;

    % Get world coordinates of bounding box of frontal image
    x_frontal_top_left_w = calib_config.width_fp/2 - array_frontal_width_w/2;
    y_frontal_top_left_w = calib_config.height_fp/2 - array_frontal_height_w/2;

    x_frontal_bottom_right_w = x_frontal_top_left_w + array_frontal_width_w;
    y_frontal_bottom_right_w = y_frontal_top_left_w + array_frontal_height_w;

    % Get world coordinates of pixels in frontal image
    [y_frontal_w, x_frontal_w] = ndgrid(linspace(y_frontal_top_left_w,y_frontal_bottom_right_w,array_frontal_height), ...
                                        linspace(x_frontal_top_left_w,x_frontal_bottom_right_w,array_frontal_width));

    % Apply transform to bring frontal image world coordinates into
    % distorted pixel space
    p_frontal_m = alg.p_m(A, ...
                          distortion, ...
                          R, ...
                          t, ...
                          [x_frontal_w(:) y_frontal_w(:)]);

    % Resample array to bring it into frontal space
    array_frontal = alg.array_interp(array, ...
                                     p_frontal_m, ...
                                     'cubic');

    array_frontal = reshape(array_frontal, ...
                            array_frontal_height, ...
                            array_frontal_width);

    % Convert board points to frontal pixel space
    p_cb_ws = alg.p_cb_w(calib_config);
    board_points_x_w = p_cb_ws(:,1);
    board_points_y_w = p_cb_ws(:,2);

    board_points_x_frontal_p = (board_points_x_w-x_frontal_top_left_w)./array_frontal_width_w*(array_frontal_width-1)+1;
    board_points_y_frontal_p = (board_points_y_w-y_frontal_top_left_w)./array_frontal_height_w*(array_frontal_height-1)+1;

    % Refine points in frontal space
    switch calib_config.calibration_target
        case 'checker'
            board_points_frontal_p_refined = alg.refine_checkers([board_points_x_frontal_p, board_points_y_frontal_p], ...
                                                                 array_frontal, ...
                                                                 alg.homography(p_cb_ws,[board_points_x_frontal_p board_points_y_frontal_p],calib_config), ...
                                                                 calib_config);
        otherwise
            error(['Calibration target: "' calib_config.calibration_target '" is not supported.']);
    end

    % Convert refined points back to world coordinates
    board_points_frontal_p_refined_x = board_points_frontal_p_refined(:,1);
    board_points_frontal_p_refined_y = board_points_frontal_p_refined(:,2);

    p_cb_ws_refined_x = (board_points_frontal_p_refined_x-1)./(array_frontal_width-1)*array_frontal_width_w+x_frontal_top_left_w;
    p_cb_ws_refined_y = (board_points_frontal_p_refined_y-1)./(array_frontal_height-1)*array_frontal_height_w+y_frontal_top_left_w;

    % Re-project world coordinate points to distorted pixel coordinates
    board_points_p_refined = alg.p_m(A, ...
                                     distortion, ...
                                     R, ...
                                     t, ...
                                     [p_cb_ws_refined_x p_cb_ws_refined_y]);
end
