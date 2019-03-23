function imgss = validate_multi_imgs(pathss)
    % Validates paths for images to multi calibration.
    %
    % Inputs:
    %   pathss - cell; paths to image files
    %
    % Outputs:
    %   imgss - cell; class.img.base img objects

    % Validate each separately
    for i = 1:numel(pathss)
        imgss{i} = intf.validate_single_imgs(pathss{i}); %#ok<AGROW>
    end

    % Make sure there are the same number of elements
    num_1 = numel(imgss{1});
    for i = 2:numel(pathss)
        num_i = numel(imgss{i});
        if num_1 ~= num_i
            error(['Number of camera 1 images: "' num2str(num_1) ...
                   '" must equal number of camera ' num2str(i) ' ' ...
                   'images: "' num2str(num_i) '"']);
        end        
    end

    % Make sure they are the same size
    s_1 = [imgss{1}(1).get_height() imgss{1}(1).get_width()];
    for i = 2:numel(pathss)
        s_i = [imgss{i}(1).get_height() imgss{i}(1).get_width()];
        if ~isequal(s_1, s_i)
            error(['Size of camera 1 images: [' num2str(s_1) '] doesnt ' ...
                   'equal size of camera ' num2str(i) ' images: [' ...
                   num2str(s_i) '].']);
        end
    end
end
