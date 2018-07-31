function write_array(array,name,file_path)
    % Writes array to data file. Has format:
    %   name = 
    %   array
    % This will append to the file and writes with 16 significant figures 
    % in ascii format, which allows it to be human readable/modifiable 
    % while also retaining double precision.
    % 
    % Inputs:
    %   array - array; array to write to file.
    %   name - string; This is the name associated with the array.
    %   file_path - string; path to file to write array to.
      
    % Write name
    f = fopen(file_path,'a');
    fprintf(f,['%s = ' newline],name);
    fclose(f);
    
    % Write array
    dlmwrite(file_path,array,'-append','delimiter',' ','precision',16);
end
