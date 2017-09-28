function write_single_calib(cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,homographies_refine,cal_config,file_path,suffix,append_calib)
    % Writes outputs of single calibration to a file, so it can be read
    % again later.
    % 
    % Inputs:
    %   cb_imgs - class.img; calibration board images
    %   board_points_ps - cell; Mx1 cell array of calibration board points 
    %   four_points_ps - cell; cell array of four points
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]    
    %   distortion - array; 4x1 array of distortions (radial and 
    %       tangential) stored as: 
    %       [beta_1; beta_2; beta_3; beta_4]
    %   rotations - cell; Mx1 cell array containing 3x3 rotation matrices
    %   translations - cell; Mx1 cell array containing 3x1 translation
    %       vectors
    %   homographies_refine - cell; cell array of homographies used for
    %       subpixel checkerboard corner refinement. Used for debugging.
    %   cal_config - struct; this is the struct returned by
    %       util.load_cal_config()
    %   file_path - string; path to file to write calibration output to
    %   suffix - string; optional suffix to add to names. This is typically
    %       used to write stereovision params by appending "_L" and "_R" as
    %       suffixes
    %   append_calib - logical; optional parameter to append another
    %       calibration to output file. If defined and set to true, this
    %       will append the calibration but skip the cal_config, as it is
    %       assumed it is getting appended to another calibration output 
    %       and that the cal_configs are the same.
    
    if ~exist('suffix','var')
        suffix = '';
    end
    
    if ~exist('append_calib','var') || ~append_calib
        % This will clear the file
        f = fopen(file_path,'w');
        fclose(f);
                
        % Write cal_config
        util.write_comment('cal_config',file_path);
        cal_config_fields = fields(cal_config);
        for i = 1:length(cal_config_fields)
            param = cal_config.(cal_config_fields{i});
            if ischar(param)
                % Must be string
                util.write_string(param,cal_config_fields{i},file_path);
            elseif isnumeric(param) && isscalar(param)
                % Must be number
                util.write_num(param,cal_config_fields{i},file_path);
            else
                error(['A param was found in cal_config that was not a ' ...
                       'string or a number, please update ' ...
                       'write_single_calib()']);
            end
        end
        util.write_newline(file_path);
    end
        
    % Write A    
    util.write_array(A,['A' suffix],file_path);
    util.write_newline(file_path);
    
    % Write distortion
    util.write_array(distortion,['distortion' suffix],file_path);
    util.write_newline(file_path);
    
    % Write stuff per calibration image; transpose anything with points so
    % they do not take a lot of vertical space.
    for i = 1:length(cb_imgs)
        util.write_comment(['Calibration' suffix ' ' num2str(i)],file_path);
        util.write_string(cb_imgs(i).get_path(),['cb_imgs' suffix],file_path);
        util.write_array(board_points_ps{i}',['board_points_ps' suffix],file_path);
        util.write_array(four_points_ps{i}',['four_points_ps' suffix],file_path);
        util.write_array(rotations{i},['rotations' suffix],file_path);
        util.write_array(translations{i},['translations' suffix],file_path);
        util.write_array(homographies_refine{i},['homographies_refine' suffix],file_path);
        util.write_newline(file_path);
    end
end