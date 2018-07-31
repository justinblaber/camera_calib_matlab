function test_write_string
    % Get temporary file
    temp_path = tempname;

    % Write a test string to this file
    s = 'string';
    name = 'test';
    util.write_string(s,name,temp_path);

    % Check file contents
    assert(strcmp(fileread(temp_path),[name ' = ' s newline]));

    % Remove temporary file
    delete(temp_path);
end