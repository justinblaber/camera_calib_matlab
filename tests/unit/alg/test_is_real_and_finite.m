function test_is_real_and_finite
    assert(alg.is_real_and_finite(2) == true);
    assert(alg.is_real_and_finite('a') == false);
end
