function test_dp_dH_c2e
    r_1 = 2;

    p_1s = [0 0;
            0 1;
            1 0;
            1 1];

    H_12 = [ 0.486486486486487   0.105405405405405   0.100000000000000;
            -0.216216216216216   0.521621621621622   0.300000000000000;
            -0.162162162162162   0.027027027027027   1.000000000000000];

    % Get finite difference approximation
    delta = 1e-10;
    dp_dH = zeros(numel(p_1s), 9);
    for i = 1:9 % Cycle over homography components
        % Increment homography component
        H_12_delta = H_12;
        H_12_delta(i) = H_12(i) + delta;

        % Finite difference approximation
        dp_dH(:, i) = reshape(((alg.apply_homography_c2e(p_1s, H_12_delta, r_1) - alg.apply_homography_c2e(p_1s, H_12, r_1))./delta)', [], 1);
    end

    % Assert
    assert(all(all(abs(dp_dH - alg.dp_dH_c2e(p_1s, H_12, r_1)) < 1e-4)));
end
