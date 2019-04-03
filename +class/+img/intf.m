classdef (Abstract) intf < handle
    % This is the interface class for an image

    methods(Abstract, Access = public)
        success = exist(obj)
        name = get_name(obj)
        array_gs = get_array_gs(obj)
        s = get_size(obj)
        h = imshow(obj, varargin)
        write(obj, name, file_path)
    end
end
