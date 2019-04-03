function imgs = validate_stereo_path_imgs(paths)
    % Validates paths for stereo images.
    %
    % Inputs:
    %   paths - struct;
    %       .L and .R - paths to image files
    %
    % Outputs:
    %   imgs - class.img.path; Mx2 img objects

    % Make sure there are the same number of elements
    num_L = numel(paths.L);
    num_R = numel(paths.R);
    if num_L ~= num_R
        error(['Number of left images: "' num2str(num_L) '" must ' ...
               'equal number of right images: "' num2str(num_R) '"']);
    end

    % Merge and validate
    imgs = intf.validate_multi_path_imgs([paths.L(:) paths.R(:)]);
end
