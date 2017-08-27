function [A,distortion,rotations,translations,board_points_is,R_s,t_s] = stereo_calibrate(cb_imgs,four_points_is,cb_config)

    % Perform single calibration on left and right -----------------------%
    [A.L,distortion.L,rotations.L,translations.L,board_points_is.L] = alg.single_calibrate(cb_imgs.L, ...
                                                                                           four_points_is.L, ...
                                                                                           cb_config);

    [A.R,distortion.R,rotations.R,translations.R,board_points_is.R] = alg.single_calibrate(cb_imgs.R, ...
                                                                                           four_points_is.R, ...
                                                                                           cb_config);
    % Perform stereo refinement ------------------------------------------%

    % Get least squares linear guess for R_s
    r = [];
    R = [];
    for i = 1:length(rotations.L)
        r = vertcat(r,rotations.R{i}(:)); %#ok<AGROW>
        R = vertcat(R,[rotations.L{i}(1,1)*eye(3) rotations.L{i}(2,1)*eye(3) rotations.L{i}(3,1)*eye(3);
                       rotations.L{i}(1,2)*eye(3) rotations.L{i}(2,2)*eye(3) rotations.L{i}(3,2)*eye(3);
                       rotations.L{i}(1,3)*eye(3) rotations.L{i}(2,3)*eye(3) rotations.L{i}(3,3)*eye(3)]); %#ok<AGROW>
    end

    % Get least squares approximation
    R_s = reshape(pinv(R)*r,3,3);

    % R_s is not necessarily orthogonal, so get the best rotational 
    % approximation.
    R_s = alg.approx_rot(R_s);

    % Get least squares linear guess for t
    t = [];
    T = [];
    for i = 1:length(rotations.L)
        t = vertcat(t,translations.R{i}-rotations.R{i}*rotations.L{i}'*translations.L{i}); %#ok<AGROW>
        T = vertcat(T,eye(3)); %#ok<AGROW>
    end

    % Get leasts squares approximation
    t_s = pinv(T)*t;

    % Perform nonlinear refinement of all parameters ---------------------%
    % Initialize distortion parameters to zero

    % Perform full optimization
    disp('--------------------------------------------');
    [A,distortion,rotations,translations,R_s,t_s] = alg.refine_stereo_params(A, ...
                                                                             distortion, ...
                                                                             rotations, ...
                                                                             translations, ...
                                                                             R_s, ...
                                                                             t_s, ....
                                                                             board_points_is, ...
                                                                             'full', ...
                                                                             cb_config);
end