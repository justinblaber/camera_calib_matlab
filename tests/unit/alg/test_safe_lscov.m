function test_safe_lscov
    % Assert
    assert(alg.safe_lscov(1, 1, 1) == 1);
    assert(isnan(alg.safe_lscov(1, 1, nan)));
    assert(isempty(alg.safe_lscov([], [])));
end
