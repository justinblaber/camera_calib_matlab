function test_dp_2_dp_1_c2e
    p_1s = [0 0;
            0 1;
            1 0;
            1 1];

    H_12 = [ 0.486486486486487   0.105405405405405   0.100000000000000;
            -0.216216216216216   0.521621621621622   0.300000000000000;
            -0.162162162162162   0.027027027027027   1.000000000000000];

    r_1 = 1;

    % Get finite difference approximation
    p_2s = alg.apply_homography_c2e(p_1s, H_12, r_1);
    delta = 1e-5;
    dp_2_dp_1_finite = sparse(8, 8);
    for i = 1:size(p_1s, 1)
        p_1_delta = p_1s(i, :);
        p_1_delta(1) = p_1_delta(1) + delta;

        p_2_delta = alg.apply_homography_c2e(p_1_delta, H_12, r_1);
        dp_2_dp_1_finite(2*i-1:2*i, 2*i-1) = (p_2_delta-p_2s(i, :))./delta; %#ok<SPRIX>

        p_1_delta = p_1s(i, :);
        p_1_delta(2) = p_1_delta(2) + delta;

        p_2_delta = alg.apply_homography_c2e(p_1_delta, H_12, r_1);
        dp_2_dp_1_finite(2*i-1:2*i, 2*i) = (p_2_delta-p_2s(i, :))./delta; %#ok<SPRIX>
    end

    % Assert
    assert(all(all(abs(dp_2_dp_1_finite - alg.dp_2_dp_1_c2e(p_1s, H_12, r_1)) < 1e-5)));
end
