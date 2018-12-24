function test_write_data
    % Get temporary file
    temp_path = tempname;

    data.a = 1;
    data.b = 'test';
    data.c = [1 2;
              3 4];

    util.write_data(data, temp_path);

    % Check file contents
    assert(strcmp(fileread(temp_path), ['a = 1' newline ...
                                        'b = test' newline ...
                                        'c = ' newline ...
                                        '1 2' newline ...
                                        '3 4' newline]));

    % Remove temporary file
    delete(temp_path);
end
