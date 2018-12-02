function write_string(s,name,file_path)
    % Writes string to data file. Has format:
    %   name = s
    % This will append to the file.
    % 
    % Inputs:
    %   s - string; string to write to file.
    %   name - string; This is the name associated with the string.
    %   file_path - string; path to file to write string to.
      
    % Write string
    f = fopen(file_path,'a');
    fprintf(f,['%s = %s' newline],name,s);
    fclose(f);
end