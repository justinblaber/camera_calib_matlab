function test_is_num
    assert(util.is_num(2) == true);
    assert(util.is_num('a') == false);
end
