function test_is_even
    assert(util.is_even(2) == true);
    assert(util.is_even(1) == false);
    assert(util.is_even(1.5) == false);
end