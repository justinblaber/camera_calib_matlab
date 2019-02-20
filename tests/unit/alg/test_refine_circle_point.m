function test_refine_circle_point
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_circle_point.mat'));

    opts.target_optimization = 'edges';
    opts.refine_ellipse_edges_h2_init = 0.75;
    opts.refine_ellipse_edges_it_cutoff = 20;
    opts.refine_ellipse_edges_norm_cutoff = 1e-6;
    [p_cb_p, cov_cb_p] = alg.refine_circle_point(p_cb_p_init, ...
                                                 boundary_p_center, ...
                                                 array_cb, ...
                                                 array_cb_dx, ...
                                                 array_cb_dy, ...
                                                 opts);

    %{
    % Plot example
    f = figure;
    imshow(array_cb, []);
    hold on;
    plot(p_cb_p_init(1), p_cb_p_init(2), 'rs');
    plot(p_cb_p(1), p_cb_p(2), 'gs');
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_cb_p - 1.0e+02 * [0.472536045090068   4.586135830011939]) < 1e-4)));
    assert(all(all(abs(cov_cb_p - 1.0e-04 * [0.305699412205980  -0.039657431351037;
                                            -0.039657431351037   0.341407540539564]) < 1e-4)));
                                         
 	clear
    
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_circle_point.mat'));

    opts.target_optimization = 'dualconic';
    [p_cb_p, cov_cb_p] = alg.refine_circle_point(p_cb_p_init, ...
                                                 boundary_p_center, ...
                                                 array_cb, ...
                                                 array_cb_dx, ...
                                                 array_cb_dy, ...
                                                 opts);

    %{
    % Plot example
    f = figure;
    imshow(array_cb, []);
    hold on;
    plot(p_cb_p_init(1), p_cb_p_init(2), 'rs');
    plot(p_cb_p(1), p_cb_p(2), 'gs');
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_cb_p - 1.0e+02 * [0.472510247867144   4.586069781632518]) < 1e-4)));
    assert(all(all(abs(cov_cb_p - [1 0;
                                   0 1]) < 1e-4)));
end
