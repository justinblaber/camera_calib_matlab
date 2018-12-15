function test_is_pos
    assert(alg.is_pos(-1) == false);
    assert(alg.is_pos(0) == false);
    assert(alg.is_pos(1) == true);
end
