function test_dr_deuler
    euler = [-0.284978769837712;
              0.262166472859945;
             -1.516780216333023];

    r = reshape(alg.euler2rot(euler), [], 1);

    % Get finite difference approximation
    delta = 1e-5;
    dr_deuler = zeros(9, 3);
    for i = 1:3
        euler_delta = euler;
        euler_delta(i) = euler(i) + delta;
        r_delta = reshape(alg.euler2rot(euler_delta), [], 1);
        dr_deuler(:, i) = (r_delta-r)./delta;
    end

    % Assert
    assert(all(all(abs(dr_deuler - alg.dr_deuler(euler)) < 1e-4)));
end
