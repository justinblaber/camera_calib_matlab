function [A,distortion,rotations,translations,R_s,t_s,board_points_ps,homographies_refine] = stereo_calibrate(cb_imgs,four_points_ps,cal_config)
    % Performs stereo calibration (mostly from Zhang's paper, some stuff
    % adapted from Bouguet's toolbox, and some stuff I've added myself) 
    % given calibration board images for the left and right camera, four 
    % point boxes around the calibration board images for the left and
    % right camera, and the calibration board config.
    %
    % Inputs:
    %   cb_imgs - struct; contains:
    %       .L - class.img; calibration board images for the left camera
    %       .R - class.img; calibration board images for the right camera
    %   four_points_ps - struct; contains:
    %       .L - cell; cell array of four points for the left camera
    %       .R - cell; cell array of four points for the right camera
    %   cal_config - struct; this is the struct returned by
    %       util.load_cal_config()
    %
    % Outputs:
    %   A - struct; contains:
    %       .L - array; optimized camera matrix for the left camera
    %       .R - array; optimized camera matrix for the right camera
    %   distortion - struct; contains:
    %       .L - array; 4x1 array of optimized distortions for the left
    %           camera
    %       .R - array; 4x1 array of optimized distortions for the right
    %           camera
    %       stored as: 
    %           [beta1; beta2; beta3; beta4]  
    %   rotations - struct; contains:
    %       .L - cell; optimized rotations for the left camera
    %       .R - cell; optimized rotations for the right camera
    %   translations - struct; contains:
    %       .L - cell; optimized translations for the left camera
    %       .R - cell; optimized translations for the right camera
    %   R_s - array; 3x3 rotation matrix describing rotation from the left
    %       camera to the right camera
    %   t_s - array; 3x1 translation vector describing translation from the
    %       left camera to the right camera
    %   board_points_ps - struct; contains:
    %       .L - cell; cell array of optimized subpixel calibration board 
    %           points for the left camera.
    %       .R - cell; cell array of optimized subpixel calibration board 
    %           points for the right camera.
    %   homographies_refine - struct; contains:
    %       .L - cell; cell array of homographies used for subpixel 
    %           checkerboard corner refinement for the left camera image
    %       .R - cell; cell array of homographies used for subpixel 
    %           checkerboard corner refinement for the right camera image
                                               
    disp('--------------------------------------------'); 
    disp('Performing stereo calibration...');     
    
    % Get number of boards
    num_boards = length(cb_imgs.L);
    
    % Perform single calibration on the left and right camera ------------%                                                                
    disp('---------');
    disp('Calibrating left camera...');    
    [A.L,distortion.L,rotations.L,translations.L,board_points_ps.L,homographies_refine.L] = alg.single_calibrate(cb_imgs.L, ...
                                                                                                                 four_points_ps.L, ...
                                                                                                                 cal_config);
                                                                  
    disp('---------');
    disp('Calibrating right camera...');  
    [A.R,distortion.R,rotations.R,translations.R,board_points_ps.R,homographies_refine.R] = alg.single_calibrate(cb_imgs.R, ...
                                                                                                                 four_points_ps.R, ...
                                                                                                                 cal_config);
    % Perform stereo refinement ------------------------------------------%
    % Get least squares linear initial guess for R_s
    r = [];
    R = [];
    for i = 1:num_boards
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
    for i = 1:num_boards
        t = vertcat(t,translations.R{i}-rotations.R{i}*rotations.L{i}'*translations.L{i}); %#ok<AGROW>
        T = vertcat(T,eye(3)); %#ok<AGROW>
    end

    % Get leasts squares approximation
    t_s = pinv(T)*t;

    % Perform nonlinear refinement of all parameters ---------------------%
    % Perform full optimization
    disp('---');
    disp('Refining full stereo parameters...');
    [A,distortion,rotations,translations,R_s,t_s] = alg.refine_stereo_params(A, ...
                                                                             distortion, ...
                                                                             rotations, ...
                                                                             translations, ...
                                                                             board_points_ps, ...
                                                                             R_s, ...
                                                                             t_s, ....
                                                                             'full', ...
                                                                             cal_config);  
                                               
    disp('---');                           
    disp('Stereo refined intrinsic params (left): ')
    disp(A.L)            
    disp('Stereo refined intrinsic params (right): ')
    disp(A.R) 
end