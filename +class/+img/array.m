classdef array < class.img.intf
    % This is the array class definition for an image.

    properties(Access = private)
        img_array
    end

    methods(Access = protected)
        function img_array = get_array(obj)
            img_array = obj.img_array;
        end
    end

    methods(Access = public)
        function obj = array(img_array)
            obj.img_array = img_array;
        end

        % Abstract methods -----------------------------------------------%

        function success = exist(obj) %#ok<MANU>
            success = true;
        end

        function name = get_name(obj) %#ok<MANU>
            name = 'N/A';
        end

        function array_gs = get_array_gs(obj)
            array_gs = obj.get_array();
        end

        function s = get_size(obj)
            s = size(obj.get_array());
        end

        function h = imshow(obj, varargin)
            h = imshow(obj.get_array(), varargin{:});
        end

        function write(obj, name, file_path)
            util.write_array(obj.get_array(), name, file_path);
        end
    end
end
