function imgs = validate_single_path_imgs(paths)
    % Validates paths for single images.
    %
    % Inputs:
    %   paths - cell; Mx1 paths to image files
    %
    % Outputs:
    %   imgs - class.img.path; Mx1 img objects

    % Validate images
    imgs = intf.validate_multi_path_imgs(paths(:));
end
