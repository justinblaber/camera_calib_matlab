function [cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,homographies_refine,calib] = parse_single_calib(calib,suffix)
    % Helper function to parse a single calibration from input calib struct
    %
    % Inputs:
    %   calib - struct; 
    %   suffix - string; optional suffix to add to names. This is typically
    %       used to write stereovision params by appending "_L" and "_R" as
    %       suffixes
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
    %   calib - struct; remainder after parsing out single calibration
                   
    % TODO: add checks for validation
    
    if ~exist('suffix','var')
        suffix = '';
    end
    
    % Must validate cb_imgs
    [cb_imgs, calib] = read_and_remove(calib,['cb_imgs' suffix]);
    cb_imgs = cell_check(cb_imgs);
    cb_imgs = class.img.validate_similar_imgs(cb_imgs);
    
    % must transpose board_points_ps
    [board_points_ps, calib] = read_and_remove(calib,['board_points_ps' suffix]);
    board_points_ps = cell_check(board_points_ps);
    for i = 1:length(board_points_ps)
        board_points_ps{i} = board_points_ps{i}';
    end 
    
    % must transpose four_points_ps
    [four_points_ps, calib] = read_and_remove(calib,['four_points_ps' suffix]);
    four_points_ps = cell_check(four_points_ps);
    for i = 1:length(four_points_ps)
        four_points_ps{i} = four_points_ps{i}';
    end
    
    [A, calib] = read_and_remove(calib,['A' suffix]);    
    [distortion, calib] = read_and_remove(calib,['distortion' suffix]);
    
    [rotations, calib] = read_and_remove(calib,['rotations' suffix]);
    rotations = cell_check(rotations);
    
    [translations, calib] = read_and_remove(calib,['translations' suffix]);
    translations = cell_check(translations);
    
    [homographies_refine, calib] = read_and_remove(calib,['homographies_refine' suffix]);
    homographies_refine = cell_check(homographies_refine);
end

function [param, calib] = read_and_remove(calib,field)
    param = calib.(field);
    calib = rmfield(calib,field);
end

function param = cell_check(param)
    if ~iscell(param)
        param = {param};
    end
end