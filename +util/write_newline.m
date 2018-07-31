function write_newline(file_path)
    % Writes newline to data file. This will append to the file.
    % 
    % Inputs:
    %   file_path - string; path to file to write newline to.
      
    % Write newline
    f = fopen(file_path,'a');
    fprintf(f,newline);
    fclose(f);
end
