classdef path < class.img.intf
    % This is the path class definition for an image.

    properties(Access = private)
        img_path
    end

    methods(Static, Access = public)
        function imgs = validate_similar_imgs(img_paths)
            % This function will make sure all image paths in the cell
            % array "img_paths" exist, are the same size, have valid
            % imfinfos, have valid colortypes and then returns all images
            % as base img objects.

            % Initialize imgs
            imgs = class.img.path.empty();
            if isempty(img_paths)
                return
            end

            % Set images
            for i = 1:numel(img_paths)
                imgs(i) = class.img.path(img_paths{i});
            end

            % Make sure all imgs exist, have valid imfinfos, and valid
            % colortypes
            for i = 1:numel(imgs)
                imgs(i).validate_exist();
                imgs(i).validate_imfinfo();
                imgs(i).validate_colortype();
            end

            % Make sure all images are the same size
            for i = 1:numel(imgs)
                if ~isequal(imgs(1).get_size(), imgs(i).get_size())
                    error(['Expected all images to be the same size, but ' ...
                           'image: ' imgs(i).get_path() ' has size of [' ...
                           num2str(imgs(i).get_size()) '] while image: ' ...
                           imgs(1).get_path() ' has a size ' ...
                           'of [' num2str(imgs(1).get_size()) '].']);
                end
            end
        end
    end

    methods(Access = protected)
        function img_path = get_path(obj)
            img_path = obj.img_path;
        end

        function img_info = get_imfinfo(obj)
            obj.validate_exist();

            img_info = imfinfo(obj.get_path());
        end

        function validate_exist(obj)
            if ~obj.exist()
                error(['Image: ' obj.get_path() ' does not exist.']);
            end
        end

        function validate_imfinfo(obj)
            img_info = obj.get_imfinfo();
            if numel(img_info) ~= 1
                error(['Image: ' obj.get_path() ' does not contain a ' ...
                       'single image. Only single image formats are ' ...
                       'supported.']);
            end
        end

        function validate_colortype(obj)
            img_info = obj.get_imfinfo();
            switch img_info.ColorType
                case {'grayscale', 'truecolor'}
                otherwise
                    error(['Image: ' obj.get_path() ' has invalid ' ...
                           'ColorType of: ' img_info.ColorType ', ' ...
                           'which is not yet supported.']);
            end
        end
    end

    methods(Access = public)
        function obj = path(img_path)
            obj.img_path = img_path;
        end

        % Abstract methods -----------------------------------------------%

        function success = exist(obj)
            success = exist(obj.get_path(), 'file') ~= 0;
        end

        function name = get_name(obj)
            name = obj.get_path();
        end

        function array_gs = get_array_gs(obj)
            % This function returns the image as a double precision
            % grayscale array
            obj.validate_exist();
            obj.validate_imfinfo();
            obj.validate_colortype();

            % Read array
            array = imread(obj.get_path());

            % Convert to double precision gray scale
            img_info = obj.get_imfinfo();
            switch img_info.ColorType
                case 'grayscale'
                    array_gs = im2double(array);
                case 'truecolor'
                    array_gs = rgb2gray(im2double(array));
            end
        end

        function s = get_size(obj)
            obj.validate_exist();
            obj.validate_imfinfo();

            img_info = obj.get_imfinfo();
            s = [img_info.Height img_info.Width];
        end

        function h = imshow(obj, varargin)
            obj.validate_exist();
            obj.validate_imfinfo();
            obj.validate_colortype();

            h = imshow(obj.get_array_gs(), varargin{:});
        end

        function write(obj, name, file_path)
            util.write_string(obj.get_path(), name, file_path);
        end
    end
end
