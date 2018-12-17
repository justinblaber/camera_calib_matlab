function test_a2A
    a = [500 100 200]';

    A = [500 0   100;
         0   500 200;
         0   0   1];

    % Assert
    assert(isequal(alg.a2A(a), A));
end
