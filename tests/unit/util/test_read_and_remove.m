function test_read_and_remove
    s.a = 1;
    s.b = 2;

    [a, s] = util.read_and_remove(s, 'a');

    % Assert
    assert(a == 1);
    assert(isequal(s, struct('b', 2)));
end
