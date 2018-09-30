function test_write_comment
    % Get temporary file
    temp_path = tempname;

    % Write a test comment to this file
    comment = 'test';
    util.write_comment(comment,temp_path);

    % Check file contents
    assert(strcmp(fileread(temp_path),['% ' comment newline]));

    % Remove temporary file
    delete(temp_path);
end
