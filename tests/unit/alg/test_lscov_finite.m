function test_lscov_finite       
    % Assert
    assert(alg.lscov_finite(1,1,1) == 1);
    assert(isnan(alg.lscov_finite(1,1,nan)));
end
