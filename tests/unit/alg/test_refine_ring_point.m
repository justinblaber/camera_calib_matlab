function test_refine_ring_point
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_ring_point.mat'));

    opts.target_optimization = 'opencv';
    [p_p, cov_p] = alg.refine_ring_point(p_p_init, ...
                                         boundary_p, ...
                                         array, ...
                                         array_dx, ...
                                         array_dy, ...
                                         opts);

    %{
    % Plot example
    f = figure;
    imshow(array, []);
    hold on;
    plot(boundary_p([1 2 3 4 1], 1), boundary_p([1 2 3 4 1], 2), '-r');
    plot(p_p_init(1), p_p_init(2), 'rs');
    plot(p_p(1), p_p(2), 'gs');
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_p - 1.0e+02 * [1.283957357300145   5.129550364241935]) < 1e-4)));
    assert(all(all(abs(cov_p - [ 0.001072118776111  -0.000012856422508;
                                -0.000012856422508   0.000981387442459]) < 1e-4)));
end
