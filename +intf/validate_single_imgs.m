function imgs = validate_single_imgs(paths)
    % Validates paths for images to single calibration.
    %
    % Inputs:
    %   paths - cell; paths to image files
    %
    % Outputs:
    %   imgs - class.img.base; img objects

    imgs = class.img.base.validate_similar_imgs(paths);
end
