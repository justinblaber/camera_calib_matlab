function test_safe_mvnpdf
    % Assert
    assert(abs(alg.safe_mvnpdf(1, 1, sparse(1)) - 0.398942280401433) < 1e-4);
    assert(isnan(alg.safe_mvnpdf(1, 1, sparse(0))));
    assert(isempty(alg.safe_mvnpdf([], [])));
end
