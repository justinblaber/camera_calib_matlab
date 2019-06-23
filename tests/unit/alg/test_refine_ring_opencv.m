function test_refine_ring_opencv
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'ring.mat'));

    [p, cov_p] = alg.refine_ring_opencv(array_dx, array_dy);

    %{
    % Plot example
    f = figure;
    imshow(array_dx.^2 + array_dy.^2, []);
    hold on;
    plot(p(1), p(2), 'gs');
    pause(1);
    close(f);
    %}

    % Assert
    assert(all(all(abs(p - [31.259934230242195  31.306502243588710]) < 1e-4)));
    assert(all(all(abs(cov_p - [ 0.016656449760357  -0.000484580487244
                                -0.000484580487244   0.013811261955077]) < 1e-4)));
end
