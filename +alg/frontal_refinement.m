function board_points_p_refined = frontal_refinement(array,A,distortion,R,t,calib_config)

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
    x_frontal_top_left_w = calib_config.four_point_width/2 - array_frontal_width_w/2;
    y_frontal_top_left_w = calib_config.four_point_height/2 - array_frontal_height_w/2;

    x_frontal_bottom_right_w = x_frontal_top_left_w + array_frontal_width_w;
    y_frontal_bottom_right_w = y_frontal_top_left_w + array_frontal_height_w;

    % Get world coordinates of pixels in frontal image
    [y_frontal_w, x_frontal_w] = ndgrid(linspace(y_frontal_top_left_w,y_frontal_bottom_right_w,array_frontal_height), ...
                                        linspace(x_frontal_top_left_w,x_frontal_bottom_right_w,array_frontal_width));

    % Apply transform to bring frontal image world coordinates into distorted 
    % pixel space
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
    board_points_w = alg.cb_points(calib_config);
    board_points_x_w = board_points_w(:,1);
    board_points_y_w = board_points_w(:,2);

    board_points_x_frontal_p = (board_points_x_w-x_frontal_top_left_w)./array_frontal_width_w*(array_frontal_width-1)+1;
    board_points_y_frontal_p = (board_points_y_w-y_frontal_top_left_w)./array_frontal_height_w*(array_frontal_height-1)+1;

    % Refine points in frontal space
    switch calib_config.calibration_pattern
        case 'checker'
            board_points_frontal_p_refined = alg.refine_checkers([board_points_x_frontal_p, board_points_y_frontal_p], ...
                                                                 array_frontal, ...
                                                                 alg.homography(board_points_w,[board_points_x_frontal_p board_points_y_frontal_p],calib_config), ...
                                                                 calib_config);
    end

    % Convert refined points back to world coordinates
    board_points_frontal_p_refined_x = board_points_frontal_p_refined(:,1);
    board_points_frontal_p_refined_y = board_points_frontal_p_refined(:,2);

    board_points_w_refined_x = (board_points_frontal_p_refined_x-1)./(array_frontal_width-1)*array_frontal_width_w+x_frontal_top_left_w;
    board_points_w_refined_y = (board_points_frontal_p_refined_y-1)./(array_frontal_height-1)*array_frontal_height_w+y_frontal_top_left_w;

    % Re-project world coordinate points to distorted pixel coordinates
    board_points_p_refined = alg.p_m(A, ...
                                     distortion, ...
                                     R, ...
                                     t, ...
                                     [board_points_w_refined_x board_points_w_refined_y]);
                                 
                                 
    subplot(1,2,2);                 
    imshow(array_frontal,[]) 
    hold on;
    plot(board_points_x_frontal_p,board_points_y_frontal_p,'rs')
    plot(board_points_frontal_p_refined_x,board_points_frontal_p_refined_y,'gs')   
end