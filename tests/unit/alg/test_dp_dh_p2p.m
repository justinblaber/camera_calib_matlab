function test_dp_dh_p2p
    p_1s = [0 0;
            0 1;
            1 0;
            1 1];
        
    H_12 = [ 0.486486486486487   0.105405405405405   0.100000000000000;
            -0.216216216216216   0.521621621621622   0.300000000000000;
            -0.162162162162162   0.027027027027027   1.000000000000000];

    % Get finite difference approximation
    delta = 1e-10;        
    dp_dh = zeros(numel(p_1s),9);
    for i = 1:9 % Cycle over homography components
        % Increment homography component
        H_12_delta = H_12;
        H_12_delta(i) = H_12(i) + delta;

        % Finite difference approximation
        dp_dh(:,i) = reshape(((alg.apply_homography_p2p(p_1s,H_12_delta) - alg.apply_homography_p2p(p_1s,H_12))./delta)',[],1);
    end
    
    % Assert
    assert(all(all(abs(dp_dh - alg.dp_dh_p2p(p_1s,H_12)) < 1e-5)));                     
end
