function test_safe_svd
    % Assert
    assert(isnan(alg.safe_svd(nan)));
end
