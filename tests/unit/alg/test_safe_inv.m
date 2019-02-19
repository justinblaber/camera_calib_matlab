function test_safe_inv
    % Assert
    assert(abs(alg.safe_inv(1) - 1) < 1e-4);
    assert(isnan(alg.safe_inv(nan)));
    assert(isempty(alg.safe_inv([])));
end
