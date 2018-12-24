function test_object_string
    a = 1;

    % Assert
    assert(strcmp(util.object_string(a), '1'));
end
