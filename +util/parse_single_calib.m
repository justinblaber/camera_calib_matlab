function [calib,calib_raw] = parse_single_calib(calib_raw,suffix)
    % Helper function to parse a single calibration from input raw 
    % calibration struct. Note that calib.config is not parsed out of
    % calib_raw. 
    %
    % Inputs:
    %   calib_raw - struct; raw calibration read from file. This needs to
    %       be parsed and formatted.
    %   suffix - string; optional suffix added to names.
    %
    % Outputs:
    %   calib - struct; contains:
    %       .intrin.A - array; optimized camera matrix
    %       .intrin.distortion - array; optimized distortions (radial and
    %           tangential) stored as: 
    %           [beta1; beta2; beta3; beta4]  
    %       .extrin(i).cb_img - class.img; ith calibration board image
    %       .extrin(i).rotation - array; ith optimized rotation
    %       .extrin(i).translation - array; ith optimized translation
    %       .extrin(i).four_points_p - array; ith array of four points
    %           around calibration board in pixel coordinates.
    %       .extrin(i).board_points_p - array; ith array of optimized 
    %           subpixel calibration board points in pixel coordinates.
    %       .extrin(i).debug.homography_refine - array; ith homography used
    %           for subpixel target refinement.
    %   calib_raw - struct; remainder after parsing out single calibration
                   
    % TODO: add checks for validation
    
    if ~exist('suffix','var')
        suffix = '';
    end
    
    % Intrinsics ---------------------------------------------------------%
    % Store these directly into calib
    [calib.intrin.A, calib_raw] = read_and_remove(calib_raw,['A' suffix]);    
    [calib.intrin.distortion, calib_raw] = read_and_remove(calib_raw,['distortion' suffix]);
    
    % Extrinsics ---------------------------------------------------------%
    % Read everything first
    [cb_img_paths, calib_raw] = read_and_remove(calib_raw,['cb_img' suffix]);
    cb_imgs = class.img.validate_similar_imgs(cell_check(cb_img_paths));
    [rotations, calib_raw] = read_and_remove(calib_raw,['rotation' suffix]);
    rotations = cell_check(rotations);    
    [translations, calib_raw] = read_and_remove(calib_raw,['translation' suffix]);
    translations = cell_check(translations);
    [four_points_ps, calib_raw] = read_and_remove(calib_raw,['four_points_p' suffix]);
    four_points_ps = cell_check(four_points_ps);   
    [board_points_ps, calib_raw] = read_and_remove(calib_raw,['board_points_p' suffix]);
    board_points_ps = cell_check(board_points_ps); 
    [homographies_refine, calib_raw] = read_and_remove(calib_raw,['homography_refine' suffix]);
    homographies_refine = cell_check(homographies_refine);
    
    % Cycle over everything and store into calib
    for i = 1:length(cb_imgs)
        calib.extrin(i).cb_img = cb_imgs(i);
        calib.extrin(i).rotation = rotations{i};
        calib.extrin(i).translation = translations{i};
        calib.extrin(i).four_points_p = four_points_ps{i}';   % Must transpose
        calib.extrin(i).board_points_p = board_points_ps{i}'; % Must transpose
        calib.extrin(i).debug.homography_refine = homographies_refine{i};
    end
end

function [param, calib_raw] = read_and_remove(calib_raw,field)
    param = calib_raw.(field);
    calib_raw = rmfield(calib_raw,field);
end

function param = cell_check(param)
    if ~iscell(param)
        param = {param};
    end
end
