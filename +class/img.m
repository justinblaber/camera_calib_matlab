classdef img < handle
% This is the class definition for an image file. Use get_gs() to load 
% images on demand (instead of loading all images at once) to reduce memory
% usage.
    
    properties(Access = private)
        path    % string
    end
        
    methods(Static, Access = public)
        function imgs = validate_similar_imgs(paths)
            % This function will make sure all image paths in the cell
            % array paths exist, are the same size, have valid imfinfos, 
            % have valid colortypes and then returns all images as img 
            % objects.
                                
            % Initialize imgs
            imgs = class.img.empty();         
            if isempty(paths)
                return
            end   
            
            % Set images
            for i = 1:length(paths)
                imgs(i) = class.img(paths{i});
            end
                                
            % Make sure all imgs exist, have valid imfinfos, and valid
            % colortypes
            for i = 1:length(imgs)                        
                imgs(i).validate_exist();             
                imgs(i).validate_imfinfo();
                imgs(i).validate_colortype();
            end            
                           
            % Make sure all images are the same size
            img_size = [imgs(1).get_height() imgs(1).get_width()];
            for i = 2:length(imgs)
                if ~isequal(img_size,[imgs(i).get_height() imgs(i).get_width()])
                    error(['Expected all images to be the same size, but ' ...
                           'image: ' imgs(i).get_path() ' has size of [' ...
                           num2str([imgs(i).get_height() imgs(i).get_width()]) ...
                           '] while image: ' imgs(1).get_path() ' has a size ' ...
                           'of [' num2str(img_size) '].']);
                end
            end
        end
    end
    
    methods(Access = private)
        function validate_exist(obj)
            if ~obj.exist()
                error(['Image: ' obj.get_path() ' does not exist.']);
            end
        end
        
        function validate_imfinfo(obj)
            img_info = imfinfo(obj.get_path()); 
            if length(img_info) ~= 1
                error(['Image: ' obj.get_path() ' does not contain a ' ...
                       'single image. Only single image formats are ' ...
                       'supported.']);
            end
        end
        
        function validate_colortype(obj)
            img_info = obj.get_imfinfo();            
            switch img_info.ColorType
                case 'grayscale'
                case 'truecolor'
                otherwise
                    error(['Image: ' obj.get_path() ' has invalid ' ...
                           'ColorType of: ' img_info.ColorType ', ' ...
                           'which is not yet supported.']);
            end
        end
    end
        
    methods(Access = public)
        function obj = img(path)
            obj.path = path;
        end
            
        function path = get_path(obj)
            path = obj.path;
        end
        
        function success = exist(obj)
            success = exist(obj.get_path(),'file') ~= 0;
        end
        
        function img_info = get_imfinfo(obj)  
            obj.validate_exist();
            
            img_info = imfinfo(obj.get_path()); 
        end
                        
        function img_gs = get_gs(obj)    
            % This function returns the image as double precision grayscale
            % intensities, which is most useful for image processing
            % algorithms.            
            obj.validate_exist();
            obj.validate_imfinfo();
            obj.validate_colortype();
             
            % Read file
            img_gs = imread(obj.get_path());  

            % Convert to double precision gray scale
            img_info = obj.get_imfinfo();            
            switch img_info.ColorType
                case 'grayscale'
                    img_gs = im2double(img_gs);
                case 'truecolor'
                    img_gs = rgb2gray(im2double(img_gs));
            end
        end
        
        function height = get_height(obj)            
            obj.validate_exist();
            obj.validate_imfinfo();
            
            img_info = obj.get_imfinfo();    
            height = img_info.Height;
        end
        
        function width = get_width(obj)        
            obj.validate_exist();
            obj.validate_imfinfo();
            
            img_info = obj.get_imfinfo();    
            width = img_info.Width;
        end
        
        function h = imshow(obj,varargin)   
            obj.validate_exist();
            obj.validate_imfinfo();
            obj.validate_colortype();
            
            h = imshow(obj.get_gs(),varargin{:});
        end
    end
end