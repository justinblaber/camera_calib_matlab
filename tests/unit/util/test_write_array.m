function test_write_array
    % Get temporary file
    temp_path = tempname;

    % Write a test array to this file
    array = [1 2; 3 4];
    name = 'test';
    util.write_array(array, name, temp_path);

    % Check file contents
    assert(strcmp(fileread(temp_path), [name ' = ' newline ...
                                        num2str(array(1, 1)) ' ' num2str(array(1, 2)) newline ...
                                        num2str(array(2, 1)) ' ' num2str(array(2, 2)) newline]));

    % Remove temporary file
    delete(temp_path);
end
