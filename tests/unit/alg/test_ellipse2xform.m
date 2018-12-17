function test_ellipse2xform
    e = [23.251025401256015;
         18.606973065070964;
          7.649815052476453;
          6.380726808930452;
          2.133246917273027];

    % Samples theta
    theta_samples = linspace(0, 2*pi, 10)';
    theta_samples(end) = [];

    % Get unit circle samples
    p_c = [cos(theta_samples)'; sin(theta_samples)'; ones(size(theta_samples))'];

    % Get xform
    xform = alg.ellipse2xform(e);

    % Transform unit circle to ellipse
    p_e = xform * p_c;

    %{
    % Plot example
    f = figure;
    array = zeros(62, 57);
    imshow(array, []);
    hold on;
    external.ellipse(e(3), e(4), e(5), e(1), e(2), 'r');
    plot(p_e(1, :), p_e(2, :), 'gs');
    pause(1);
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_e(1:2, :)' - [19.171678323910342  25.078340599035219;
                                      16.656437328914507  21.377184245566156;
                                      17.226877383649875  16.379815293275517;
                                      20.616083246770742  12.424558214509482;
                                      25.238207828095373  11.362121754208665;
                                      28.930507666829239  13.689631739813835;
                                      29.965314633086962  18.318020381668187;
                                      27.858431046758170  23.081613195438067;
                                      23.595691153288932  25.751472162123537;]) < 1e-4)));
end
