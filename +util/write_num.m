function write_num(num, name, file_path)
    % Writes number to data file. Has format:
    %   name = num
    % This will append to the file and writes with 16 significant figures
    % in ascii format, which allows it to be human readable/modifiable
    % while also retaining double precision.
    %
    % Inputs:
    %   num - scalar; number to write to file.
    %   name - string; This is the name associated with the number.
    %   file_path - string; path to file to write number to.

    % Write num
    f = fopen(file_path, 'a');
    fprintf(f, ['%s = %.16g' newline], name, num);
    fclose(f);
end
