function test_mvnpdf_pos_def
    % Assert
    assert(abs(alg.mvnpdf_pos_def(1,1,sparse(1)) - 0.398942280401433) < 1e-4);
    assert(isnan(alg.mvnpdf_pos_def(1,1,sparse(0))));
end