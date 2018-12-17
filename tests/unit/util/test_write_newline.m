function test_write_newline
    % Get temporary file
    temp_path = tempname;

    % Write a newline to this file
    util.write_newline(temp_path);

    % Check file contents
    assert(strcmp(fileread(temp_path), newline));

    % Remove temporary file
    delete(temp_path);
end
