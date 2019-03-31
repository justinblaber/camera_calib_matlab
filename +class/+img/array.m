classdef array < class.img.intf
    % This is the array class definition for an image.

    properties(Access = private)
        array_img
    end

    methods(Access = private)
        function array_img = get_array(obj)
            array_img = obj.array_img;
        end
    end

    methods(Access = public)
        function obj = array(array_img)
            obj.array_img = array_img;
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
