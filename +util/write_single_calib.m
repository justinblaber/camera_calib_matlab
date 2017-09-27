function write_single_calib(cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,homographies_refine,cal_config,file_path,suffix,append_calib)
    % Writes outputs of single calibration to a file, so it can be read
    % again later.
    % 
    % Inputs:
    
    if ~exist('suffix','var')
        suffix = '';
    end
    
    if ~exist('append_calib','var') || ~append_calib
        % This will clear the file
        f = fopen(file_path,'w');
        fclose(f);
                
        % Only write cal_config if we are not appending
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
    
    % Write stuff per calibration image      
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