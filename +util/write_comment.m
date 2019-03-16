function write_comment(comment, file_path)
    % Writes comment to data file. Has format:
    %
    %   % comment
    %
    % This will append to the file.
    %
    % Inputs:
    %   comment - string; comment to write
    %   file_path - string; path to file to write comment to.

    % Write comment
    f = fopen(file_path, 'a');
    fprintf(f, ['%% %s' newline], comment);
    fclose(f);
end
