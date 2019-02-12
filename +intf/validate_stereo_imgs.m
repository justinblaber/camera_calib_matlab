function imgs = validate_stereo_imgs(paths)
    % Validates paths for images to stereo calibration.
    %
    % Inputs:
    %   paths - struct;
    %       .L and .R - paths to image files
    %
    % Outputs:
    %   imgs - struct;
    %       .L and .R - class.img; img objects

    % Validate left and right separately
    imgs.L = intf.validate_single_imgs(paths.L);
    imgs.R = intf.validate_single_imgs(paths.R);
    
    % Make sure there are the same number of elements
    num_L = numel(imgs.L);
    num_R = numel(imgs.R);
    if num_L ~= num_R
        error(['Number of left images: "' num2str(num_L) '" must ' ...
               'equal number of right images: "' num2str(num_R) '"']);
    end
    
    % Make sure they are the same size
    s_L = [imgs.L(1).get_height() imgs.L(1).get_width()];
    s_R = [imgs.R(1).get_height() imgs.R(1).get_width()];
    if ~isequal(s_L, s_R)
        error(['Size of left images: [' num2str(s_L) '] doesnt equal ' ...
               'Size of right images: [' num2str(s_R) '].']);
    end
end
