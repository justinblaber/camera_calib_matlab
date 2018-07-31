function test_is_pos
    assert(util.is_pos(-1) == false);
    assert(util.is_pos(0) == false);
    assert(util.is_pos(1) == true);
end