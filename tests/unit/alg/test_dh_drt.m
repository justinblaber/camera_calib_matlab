function test_dh_drt
    a = 1.0e+02 * [5.856079564482973;
                   3.205000000000000;
                   2.405000000000000];

    t = 1.0e+03 * [-0.444199579715235;
                    0.409595472908999;
                    1.038284747668525];

    r = [ 0.052145054365658;
         -0.964422073568075;
         -0.259173604597105;
          0.954334052512317;
          0.124569345350011;
         -0.271530835108071;
          0.294155417289261;
         -0.233179206220718;
          0.926876501086125];

    rt = vertcat(r, t);

    H = alg.a2A(a)*[rt(1:3) rt(4:6) rt(10:12)];

    % Get finite difference approximation
    delta = 1e-5;
    dh_drt = zeros(9, 12);
    for i = 1:12
        rt_delta = rt;
        rt_delta(i) = rt(i) + delta;
        H_delta = alg.a2A(a)*[rt_delta(1:3) rt_delta(4:6) rt_delta(10:12)];
        H_delta = (H_delta-H)./delta;
        dh_drt(:, i) = H_delta(:);
    end

    % Assert
    assert(all(all(abs(dh_drt - alg.dh_drt(alg.a2A(a))) < 1e-4)));
end
