classdef (Abstract) intf < handle
    % This is the interface class for an image

    methods(Abstract, Access = public)
        path = get_path(obj)
        success = exist(obj)
        array_gs = get_array_gs(obj)
        height = get_height(obj)
        width = get_width(obj)
        h = imshow(obj, varargin)
    end
end
