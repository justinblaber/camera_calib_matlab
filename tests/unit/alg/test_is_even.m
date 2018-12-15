function test_is_even
    % Assert
    assert(alg.is_even(2) == true);
    assert(alg.is_even(1) == false);
    assert(alg.is_even(1.5) == false);
end
