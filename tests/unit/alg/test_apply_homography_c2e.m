function test_apply_homography_c2e
    r_1 = 2;

    p_1s = [0 0;
            0 1;
            1 0;
            1 1];

    H_12 = [ 0.486486486486487   0.105405405405405   0.100000000000000;
            -0.216216216216216   0.521621621621622   0.300000000000000;
            -0.162162162162162   0.027027027027027   1.000000000000000];

    % Ground truth
    p_2s = zeros(size(p_1s));
    for i = 1:size(p_1s, 1)
        Aqw = [ 1           0          -p_1s(i, 1);
                0           1          -p_1s(i, 2);
               -p_1s(i, 1) -p_1s(i, 2)  p_1s(i, 1)^2 + p_1s(i, 2)^2 - r_1^2];
        Aqi = inv(H_12)'*Aqw*inv(H_12);
        p_prime = inv(Aqi)*[0; 0; 1]; %#ok<MINV>
        p_2s(i, :) = [p_prime(1)/p_prime(3) p_prime(2)/p_prime(3)];
    end

    %{
    % plot
    % Get coordinates of circle
    theta = linspace(0, 2*pi, 100);
    theta = theta(1:end-1); % Remove duplicate last sample
    x_circle = r_1*cos(theta);
    y_circle = r_1*sin(theta);

    % Apply homography to circle
    p_ellipse = alg.apply_homography_p2p([x_circle' y_circle'], H_12);

    % Get projected center
    p_hc = alg.apply_homography_p2p([0 0], H_12);

    % Get ellipse center
    p_ch = p_2s(1, :);

    f = figure;
    plot(x_circle, y_circle, 'rs');
    hold on;
    plot(p_ellipse(:, 1), p_ellipse(:, 2), 'gs');
    plot(p_hc(1, 1), p_hc(1, 2), 'gx');
    plot(p_ch(1, 1), p_ch(1, 2), 'gs');
    pause(1);
    close(f);
    %}

    % Assert
    assert(all(all(abs(alg.apply_homography_c2e(p_1s, H_12, r_1) - p_2s) < 1e-4)));

    % Another check is to use a very small radius and see if this is almost
    % equivalent to applying a point to point homography
    assert(all(all(abs(alg.apply_homography_c2e(p_1s, H_12, eps('single')) - alg.apply_homography_p2p(p_1s, H_12)) < 1e-4)));
end
