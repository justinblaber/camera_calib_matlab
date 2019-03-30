function imgs = validate_single_path_imgs(paths)
    % Validates paths for images to single calibration.
    %
    % Inputs:
    %   paths - cell; paths to image files
    %
    % Outputs:
    %   imgs - class.img.path; MxN img objects

    if isempty(paths)
        error('Input paths is empty!')
    end

    % Validate images
    imgs = intf.validate_multi_path_imgs(paths(:));
end
