function imgs = validate_multi_path_imgs(paths)
    % Validates paths for images to multi calibration.
    %
    % Inputs:
    %   paths - cell; MxN paths to image files
    %
    % Outputs:
    %   imgs - class.img.base; MxN img objects

    if isempty(paths)
        error('Input paths is empty!')
    end

    % Validate images
    imgs = class.img.path.validate_similar_imgs(paths(:));
    imgs = reshape(imgs, size(paths));
end
