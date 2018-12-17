function test_bb_ellipse
    e = [23.251025401256015;
         18.606973065070964;
          7.649815052476453;
          6.380726808930452;
          2.133246917273027];

    % Get bounding box
    bb_e = alg.bb_ellipse(e);

    % Sample theta
    theta_samples = linspace(0, 2*pi, 1000)';
    theta_samples(end) = [];

    % Sample ellipse
    p_e = alg.sample_ellipse(e, theta_samples);

    % Assert
    assert(all(all(abs(bb_e - [16.485146664357991  11.295596322973051
                               30.016904138154040  25.918349807168877]) < 1e-4)));
    assert(all(all(abs(bb_e - [min(p_e);
                               max(p_e)]) < 1e-4)));
end
