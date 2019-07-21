function test_refine_ellipse_point
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_ellipse_point.mat'));

    opts.target_optimization = 'dualconic';
    opts.refine_ellipse_dualconic_it_cutoff = 20;
    opts.refine_ellipse_dualconic_norm_cutoff = 0.001;
    [p_p, cov_p] = alg.refine_ellipse_point(p_p_init, ...
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
    assert(all(all(abs(p_p - 1.0e+02 * [0.472510247867144   4.586069781632518]) < 1e-4)));
    assert(all(all(abs(cov_p - [1 0;
                                0 1]) < 1e-4)));

    clear

    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_ellipse_point.mat'));

    opts.target_optimization = 'edges';
    opts.refine_ellipse_dualconic_it_cutoff = 20;
    opts.refine_ellipse_dualconic_norm_cutoff = 0.001;
    opts.refine_ellipse_edges_h2_init = 0.75;
    opts.refine_ellipse_edges_it_cutoff = 20;
    opts.refine_ellipse_edges_norm_cutoff = 1e-6;
    [p_p, cov_p] = alg.refine_ellipse_point(p_p_init, ...
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
    assert(all(all(abs(p_p - 1.0e+02 * [0.472536045090068   4.586135830011939]) < 1e-4)));
    assert(all(all(abs(cov_p - 1.0e-04 * [0.305699412205980  -0.039657431351037;
                                          -0.039657431351037   0.341407540539564]) < 1e-4)));

    clear

    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_ellipse_point.mat'));

    opts.target_optimization = 'dot';
    opts.refine_ellipse_dualconic_it_cutoff = 20;
    opts.refine_ellipse_dualconic_norm_cutoff = 0.001;
    opts.refine_ellipse_dot_h2_init = 1;
    opts.refine_ellipse_dot_it_cutoff = 20;
    opts.refine_ellipse_dot_norm_cutoff = 1e-6;
    [p_p, cov_p] = alg.refine_ellipse_point(p_p_init, ...
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
    assert(all(all(abs(p_p - 1.0e+02 * [0.472055890287315   4.586914433264144]) < 1e-4)));
    assert(all(all(abs(cov_p - 1.0e-04 * [0.590805395297924  -0.078326529726054;
                                          -0.078326529726054   0.661363136080293]) < 1e-4)));
end
