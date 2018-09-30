function test_is_int
    assert(util.is_int(1) == true);
    assert(util.is_int(1.5) == false);
end
